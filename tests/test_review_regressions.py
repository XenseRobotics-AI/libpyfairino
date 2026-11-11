"""D-01..D-06: observable API/events, explicit fake backend and network denial only."""
import concurrent.futures
import json
import math
import os
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET
import pytest
import libpyfairino as f


def configuration(**kwargs):
    params=dict(rpc_timeout_ms=800,worker_watchdog_ms=5000)
    params.update(kwargs)
    return f.HardwareConfig(f.SDKRobotType.ART3_R7,[-180.]*14,[180.]*14,
        list(range(7))*2,[1.]*14,[0.]*14,[1.]*14,'SYNTHETIC FAKE ONLY','SYNTHETIC FAKE ONLY',**params)


@pytest.fixture
def rig(tmp_path, monkeypatch, fake_backend):
    monkeypatch.setenv('LPF_TEST_DIR',str(tmp_path))
    sessions=[]
    def create(**kwargs):
        s=f.Session(configuration(**kwargs),backend_override=fake_backend)
        s.connect(deny_network=True)
        sessions.append(s)
        return s
    yield create,tmp_path
    for session in sessions:session.close()


def events(directory):
    path=directory/'events.jsonl'
    if not path.exists():return []
    result=[]
    for line in path.read_text().splitlines():
        try:result.append(json.loads(line))
        except json.JSONDecodeError:pass  # a concurrent append may still be in progress
    return result


def wait_for(predicate, seconds=4):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.002)
    raise AssertionError('observable event did not arrive before test deadline')


def entered(d,event,arm):
    return any(x['event']==event and x['arm']==arm for x in events(d))


def servo_events(d):return [x for x in events(d) if x['event']=='servo_enter']


def activate(s):
    assert s.left.ServoMoveStart().ok
    assert s.right.ServoMoveStart().ok


def raw_codes(report):
    return [x.reply.code if x.reply is not None else None for x in report.operations]


def assert_reaped(s):
    with pytest.raises(ChildProcessError):os.waitpid(s.last_worker_pid,os.WNOHANG)


def test_D01_preflight_active_limits_and_parameters_send_neither_arm(rig):
    make,d=rig;s=make();activate(s)
    assert s.right.SetLimitPositive([1.]*7).ok
    for q,period in [([.1]*14,1),([0.]*14,0),([float('nan')]+[0.]*13,1),([math.pi+1]*14,1)]:
        with pytest.raises(ValueError):s.servo_joints(q,period)
        assert not servo_events(d)
    result=s.servo_joints([0.]*14,1)
    assert result.complete and all(x.state==f.DispatchState.SdkReplied for x in result.arms)


def test_D01_limit_update_barrier_precedes_full_pair_preflight(rig,monkeypatch):
    make,d=rig;s=make();activate(s);monkeypatch.setenv('LPF_TEST_BLOCK','limit1')
    # The worker inherits its environment at spawn, so recreate with the explicit gate.
    s.close();s=make();activate(s)
    with concurrent.futures.ThreadPoolExecutor(2) as pool:
        update=pool.submit(s.right.SetLimitPositive,[1.]*7)
        wait_for(lambda:entered(d,'limit_enter',1))
        pair=pool.submit(s.servo_joints,[.1]*14,1)
        assert not pair.done()
        (d/'release_limit1').touch()
        assert update.result(timeout=3).ok
        with pytest.raises(ValueError):pair.result(timeout=3)
    assert not servo_events(d)


def test_D01_pair_dispatch_barrier_excludes_limit_update_interleaving(rig,monkeypatch):
    monkeypatch.setenv('LPF_TEST_BLOCK','servo0');make,d=rig;s=make();activate(s)
    with concurrent.futures.ThreadPoolExecutor(2) as pool:
        pair=pool.submit(s.servo_joints,[0.]*14,1)
        wait_for(lambda:entered(d,'servo_enter',0))
        update=pool.submit(s.right.SetLimitPositive,[1.]*7)
        assert not entered(d,'limit_enter',1)
        (d/'release_servo0').touch()
        assert pair.result(timeout=3).complete
        assert update.result(timeout=3).ok
    log=events(d)
    assert next(i for i,x in enumerate(log) if x['event']=='servo_enter' and x['arm']==1) < next(i for i,x in enumerate(log) if x['event']=='limit_enter' and x['arm']==1)


@pytest.mark.parametrize('codes,states',[('-7,0',[f.DispatchState.SdkReplied,f.DispatchState.NotDispatched]),('0,-8',[f.DispatchState.SdkReplied,f.DispatchState.SdkReplied])])
def test_D01_nonzero_reply_preserves_both_arm_facts(rig,monkeypatch,codes,states):
    monkeypatch.setenv('LPF_TEST_SERVO_CODES',codes);make,d=rig;s=make();activate(s)
    result=s.servo_joints([0.]*14,1)
    assert not result.complete
    assert [x.state for x in result.arms]==states
    assert result.arms[0].reply.code==int(codes.split(',')[0])
    if states[1]==f.DispatchState.SdkReplied:assert result.arms[1].reply.code==-8
    else:assert result.arms[1].reply is None
    assert not result.physical_execution_confirmed


@pytest.mark.parametrize('failure',['block','exit','drop_reply'])
def test_D01_second_arm_ipc_uncertainty_keeps_first_reply(rig,monkeypatch,failure):
    monkeypatch.setenv({'block':'LPF_TEST_BLOCK','exit':'LPF_TEST_EXIT_AT','drop_reply':'LPF_TEST_DROP_REPLY'}[failure],'servo1')
    make,d=rig;s=make(rpc_timeout_ms=120);activate(s)
    result=s.servo_joints([0.]*14,1)
    assert result.arms[0].state==f.DispatchState.SdkReplied and result.arms[0].reply.code==0
    assert result.arms[1].state==f.DispatchState.Unknown and result.arms[1].reply is None
    assert not result.complete and not result.physical_execution_confirmed
    assert entered(d,'servo_enter',1)
    assert s.last_pair.arms[0].reply.code==0
    assert_reaped(s)


def body_setup(make,model,body,head):
    session=make(fixed_torso_rad=body,fixed_head_rad=head,cartesian_frame_evidence='SYNTHETIC MODEL ONLY')
    c=f.Controller(session,[0.]*14,period_ms=2,command_timeout_ms=10000);c.start()
    options=f.Options();options.body_avoidance=False;options.torso_position=body;options.head_position=head
    left=model.forward_kinematics([0.]*14,'left',options);right=model.forward_kinematics([0.]*14,'right',options)
    return session,c,options,left,right


@pytest.mark.parametrize('axis',range(6))
@pytest.mark.parametrize('edge',[0,1])
@pytest.mark.parametrize('position',['boundary','inside','outside'])
def test_D02_actual_model_closed_intervals_every_axis(rig,axis,edge,position):
    make,d=rig;model=f.Kinematics();bounds=list(model.torso_limits)+[model.head_limits]
    values=[(a+b)/2 for a,b in bounds];value=bounds[axis][edge]
    if position=='inside':value=math.nextafter(value,bounds[axis][1-edge])
    if position=='outside':value=math.nextafter(value,-math.inf if edge==0 else math.inf)
    values[axis]=value
    s,c,options,left,right=body_setup(make,model,values[:5],values[5])
    try:
        before=c.status.target_sequence
        if position=='outside':
            with pytest.raises(ValueError,match='closed interval'):c.submit_cartesian(model,left,right,[0.]*14,options)
            assert c.status.target_sequence==before
        else:
            result=c.submit_cartesian(model,left,right,[0.]*14,options)
            assert result.success and c.status.target_sequence==before+1
            coords=model.joint_coordinates
            for index,name in enumerate(['j20','j19','j18','j16','j15','j17']):assert result.full_q[coords[name]]==values[index]
    finally:c.stop()


@pytest.mark.parametrize('axis',range(6))
@pytest.mark.parametrize('value',[float('nan'),float('inf'),-float('inf')])
def test_D02_nonfinite_fixed_options_or_config_rejected(rig,axis,value):
    make,d=rig;model=f.Kinematics();body=[(a+b)/2 for a,b in model.torso_limits];head=sum(model.head_limits)/2
    s,c,options,left,right=body_setup(make,model,body,head)
    try:
        if axis<5:
            changed=body[:];changed[axis]=value;options.torso_position=changed
        else:options.head_position=value
        with pytest.raises(ValueError,match='closed interval'):c.submit_cartesian(model,left,right,[0.]*14,options)
        assert c.status.target_sequence==0
        if axis<5:
            with pytest.raises(ValueError):configuration(fixed_torso_rad=changed,fixed_head_rad=head,cartesian_frame_evidence='synthetic')
        else:
            with pytest.raises(ValueError):configuration(fixed_torso_rad=body,fixed_head_rad=value,cartesian_frame_evidence='synthetic')
    finally:c.stop()


@pytest.mark.parametrize('axis',range(6))
def test_D02_calibration_match_is_exact_and_separate_from_limits(rig,axis):
    make,d=rig;model=f.Kinematics();values=[(a+b)/2 for a,b in list(model.torso_limits)+[model.head_limits]]
    s,c,options,left,right=body_setup(make,model,values[:5],values[5])
    changed=values[:];changed[axis]=math.nextafter(changed[axis],math.inf)
    options.torso_position=changed[:5];options.head_position=changed[5]
    try:
        with pytest.raises(ValueError,match='differs'):c.submit_cartesian(model,left,right,[0.]*14,options)
        assert c.status.target_sequence==0
    finally:c.stop()


def test_D02_different_model_limits_and_legal_nonzero_body(rig,tmp_path):
    make,d=rig;tree=ET.parse(f.model_path());joint=next(x for x in tree.findall('joint') if x.get('name')=='j20')
    joint.find('limit').set('lower','-0.2');joint.find('limit').set('upper','0.2')
    path=tmp_path/'different-limits.urdf';tree.write(path)
    custom=f.Kinematics(urdf_path=str(path));body=[-.1,.2,.1,-.1,.1]
    s,c,options,left,right=body_setup(make,custom,body,.2)
    try:
        assert c.submit_cartesian(custom,left,right,[0.]*14,options).success
        with pytest.raises(ValueError,match='closed interval'):c.submit_cartesian(f.Kinematics(),left,right,[0.]*14,options)
        assert c.status.target_sequence==1
    finally:c.stop()


def test_D02_options_snapshot_survives_python_mutation_after_native_entry(rig):
    make,d=rig;model=f.Kinematics();body=[.1,.2,.1,-.1,.1]
    s,c,options,_,_=body_setup(make,model,body,.2)
    home=[math.radians(x) for x in [-30,45,0,60,0,0,0,30,45,0,60,0,0,0]]
    left=model.forward_kinematics(home,'left',options);right=model.forward_kinematics(home,'right',options)
    options.max_step_norm=1e-4;options.max_iterations=15000
    with concurrent.futures.ThreadPoolExecutor(1) as pool:
        future=pool.submit(c.submit_cartesian,model,left,right,[x+.02 for x in home],options)
        wait_for(lambda:c.status.cartesian_in_progress)
        options.torso_position=[100.]*5;options.head_position=100.
        result=future.result(timeout=15)
    try:
        assert result.success and c.status.target_sequence==1
        assert c.status.last_cartesian_torso==body and c.status.last_cartesian_head==.2
        for index,name in enumerate(['j20','j19','j18','j16','j15']):assert result.full_q[model.joint_coordinates[name]]==body[index]
        assert result.full_q[model.joint_coordinates['j17']]==.2
    finally:c.stop()


@pytest.mark.parametrize('new_target',[False,True])
def test_D03_captured_deadline_is_not_renewed_during_first_arm(rig,monkeypatch,new_target):
    monkeypatch.setenv('LPF_TEST_BLOCK','servo0');make,d=rig;s=make(rpc_timeout_ms=1000)
    c=f.Controller(s,[0.]*14,period_ms=2,command_timeout_ms=120);c.start()
    wait_for(lambda:entered(d,'servo_enter',0))
    if new_target:c.submit_joints([.2]*14)
    wait_for(lambda:not c.status.running)
    (d/'release_servo0').touch()
    status=c.status
    assert not entered(d,'servo_enter',1)
    assert status.last_pair.target_sequence==0
    assert status.last_pair.arms[0].state==f.DispatchState.Unknown
    assert status.last_pair.arms[1].state==f.DispatchState.NotDispatched
    assert not status.last_pair.complete and 'deadline' in status.error
    if new_target:assert status.target_sequence==1
    assert_reaped(s);assert not c.stop().physical_stop_confirmed


def test_D03_stop_request_is_observable_while_first_arm_waits(rig,monkeypatch):
    monkeypatch.setenv('LPF_TEST_BLOCK','servo0');make,d=rig;s=make(rpc_timeout_ms=2000)
    c=f.Controller(s,[0.]*14,period_ms=2,command_timeout_ms=3000);c.start()
    wait_for(lambda:entered(d,'servo_enter',0))
    with concurrent.futures.ThreadPoolExecutor(1) as pool:
        stopping=pool.submit(c.stop)
        wait_for(lambda:c.status.stop_requested)
        assert not stopping.done()
        (d/'release_servo0').touch()
        stop=stopping.result(timeout=3)
    assert not entered(d,'servo_enter',1)
    assert c.status.last_pair.arms[0].state==f.DispatchState.SdkReplied
    assert c.status.last_pair.arms[1].state==f.DispatchState.NotDispatched
    assert stop.ok and not stop.physical_stop_confirmed


def test_D03_new_target_does_not_mix_an_old_pair_with_new_values(rig,monkeypatch):
    monkeypatch.setenv('LPF_TEST_BLOCK','servo0');make,d=rig;s=make(rpc_timeout_ms=2000)
    c=f.Controller(s,[0.]*14,period_ms=2,command_timeout_ms=3000);c.start()
    wait_for(lambda:entered(d,'servo_enter',0));seq=c.submit_joints([.2]*14)
    (d/'release_servo0').touch();wait_for(lambda:c.status.sent_sequence==seq)
    c.stop();first=servo_events(d)[:2]
    assert [(x['arm'],x['value']) for x in first]==[(0,0),(1,0)]
    assert c.status.last_pair.target_sequence==seq


@pytest.mark.parametrize('index',range(4))
def test_D04_watchdog_fault_is_latched_by_arm_and_operation(rig,monkeypatch,index):
    codes=[0]*4;codes[index]=-10-index
    monkeypatch.setenv('LPF_TEST_STOP_CODES',','.join(map(str,codes)));monkeypatch.setenv('LPF_TEST_FAIL_ONCE','1')
    make,d=rig;s=make(worker_watchdog_ms=30);activate(s)
    wait_for(lambda:entered(d,'end_return',1))
    getter=s.left.GetActualJointPosDegree();assert getter.ok
    faults=s.poll_status()
    expected=f.Operation.StopMotion if index%2==0 else f.Operation.ServoMoveEnd
    assert any(x.arm==index//2 and x.operation==expected and x.sdk_code==codes[index] for x in faults)
    assert s.left.ServoMoveStart().state==f.DispatchState.WorkerRejected
    assert s.right.ServoJ([0.]*7,1).state==f.DispatchState.WorkerRejected
    c=f.Controller(s,[0.]*14)
    assert c.status.faults
    with pytest.raises(RuntimeError,match='latched'):c.start()
    retried=s.stop_all();assert all(x.ok for x in retried.operations) and not retried.ok
    assert any(x.sdk_code==codes[index] for x in retried.faults)
    cleanup=s.close();assert not cleanup.ok and cleanup.process.reaped
    assert any(x.sdk_code==codes[index] for x in cleanup.stop.faults)
    assert s.diagnostic
    s.connect(deny_network=True)  # explicit worker reconstruction starts a new fault generation
    assert not s.poll_status()
    assert s.last_cleanup.stop.faults  # preceding generation remains inspectable


@pytest.mark.parametrize('index',range(4))
def test_D04_explicit_stop_failure_survives_successful_retry(rig,monkeypatch,index):
    codes=[0]*4;codes[index]=-5-index
    monkeypatch.setenv('LPF_TEST_STOP_CODES',','.join(map(str,codes)));monkeypatch.setenv('LPF_TEST_FAIL_ONCE','1')
    make,d=rig;s=make();arm=s.left if index//2==0 else s.right
    method=arm.StopMotion if index%2==0 else arm.ServoMoveEnd
    first=method();assert first.reply.code==codes[index]
    assert method().reply.code==0
    assert s.left.GetActualJointPosDegree().ok
    assert any(x.sdk_code==codes[index] for x in s.poll_status())
    assert s.left.ServoMoveStart().state==f.DispatchState.WorkerRejected


@pytest.mark.parametrize('codes',[[-2,0,0,0],[0,-3,0,0],[0,0,-4,0],[0,0,0,-5],[-2,-3,-4,-5]])
def test_D05_close_returns_all_raw_errors_and_reaps(rig,monkeypatch,codes):
    monkeypatch.setenv('LPF_TEST_STOP_CODES',','.join(map(str,codes)));make,d=rig;s=make()
    result=s.close()
    assert not result.ok and raw_codes(result.stop)==codes
    assert result.sdk_destructors_completed and result.process.reaped and not result.process.forced
    assert len(result.stop.faults)==sum(code!=0 for code in codes) and not result.physical_stop_confirmed
    assert raw_codes(s.last_cleanup.stop)==codes
    assert_reaped(s)


def test_D05_controller_stop_returns_all_cleanup_errors(rig,monkeypatch):
    monkeypatch.setenv('LPF_TEST_STOP_CODES','-2,-3,-4,-5');make,d=rig;s=make()
    c=f.Controller(s,[0.]*14,command_timeout_ms=3000);c.start();wait_for(lambda:len(servo_events(d))>=2)
    report=c.stop()
    assert not report.ok and raw_codes(report)==[-2,-3,-4,-5]
    assert raw_codes(c.status.stop_result)==[-2,-3,-4,-5]
    assert c.status.faults and not report.physical_stop_confirmed


def test_D05_context_preserves_existing_exception_and_exposes_cleanup(rig,monkeypatch):
    monkeypatch.setenv('LPF_TEST_STOP_CODES','-2,-3,-4,-5');make,d=rig;s=make()
    with pytest.raises(ValueError,match='ORIGINAL') as original:
        with s:raise ValueError('ORIGINAL')
    assert raw_codes(s.last_cleanup.stop)==[-2,-3,-4,-5]
    assert original.value.__notes__ and s.last_cleanup.process.reaped
    s=make()
    with pytest.raises(f.CleanupError) as cleanup:
        with s:pass
    assert raw_codes(cleanup.value.result.stop)==[-2,-3,-4,-5]
    assert cleanup.value.result.process.reaped


@pytest.mark.parametrize('block',['stop1','destructor0','destructor1'])
def test_D05_D06_known_stop_errors_survive_later_hanging_phase(rig,monkeypatch,block):
    monkeypatch.setenv('LPF_TEST_STOP_CODES','-2,-3,-4,-5');monkeypatch.setenv('LPF_TEST_BLOCK',block)
    make,d=rig;s=make(rpc_timeout_ms=150)
    result=s.close()
    assert not result.ok and result.process.reaped and result.process.forced
    assert raw_codes(result.stop)[:2]==[-2,-3]
    if block=='stop1':assert all(x.state==f.DispatchState.Unknown for x in result.stop.operations[2:])
    else:assert raw_codes(result.stop)==[-2,-3,-4,-5]
    assert not result.sdk_destructors_completed and not result.physical_stop_confirmed
    assert_reaped(s)


def test_D06_normal_close_waits_for_both_destructors_and_normal_exit(rig,monkeypatch):
    monkeypatch.setenv('LPF_TEST_DTOR_SLEEP_MS','30');make,d=rig;s=make()
    result=s.close()
    assert result.ok and result.sdk_destructors_completed and result.process.reaped
    assert result.process.exit_code==0 and result.process.term_signal is None and not result.process.forced
    log=events(d)
    assert [(x['event'],x['arm']) for x in log if x['event'].startswith('destructor')]==[('destructor_enter',0),('destructor_complete',0),('destructor_enter',1),('destructor_complete',1)]
    assert_reaped(s)


@pytest.mark.parametrize('stage,known',[('stop0',0),('end0',1)])
def test_D05_cleanup_reply_loss_reports_unknown_without_overwriting_known_codes(rig,monkeypatch,stage,known):
    monkeypatch.setenv('LPF_TEST_STOP_CODES','-2,-3,-4,-5');monkeypatch.setenv('LPF_TEST_DROP_REPLY',stage)
    make,d=rig;s=make()
    result=s.close()
    assert not result.ok and result.process.reaped
    if known:assert result.stop.operations[0].reply.code==-2
    assert all(op.state==f.DispatchState.Unknown and op.reply is None for op in result.stop.operations[known:])
    assert not result.sdk_destructors_completed and not result.physical_stop_confirmed
    assert_reaped(s)


def test_D06_exit_zero_during_destruction_is_not_a_destructor_ack(rig,monkeypatch):
    monkeypatch.setenv('LPF_TEST_EXIT_AT','destructor0');make,d=rig;s=make()
    result=s.close()
    assert all(op.ok for op in result.stop.operations)
    assert not result.ok and not result.sdk_destructors_completed and result.process.reaped
    assert not entered(d,'destructor_complete',0)
    assert_reaped(s)


def test_D05_unattempted_stop_slots_still_identify_each_arm_and_operation(rig):
    make,d=rig;s=make();c=f.Controller(s,[0.]*14)
    report=c.stop()
    assert [(x.arm,x.operation,x.state,x.reply) for x in report.operations]==[
        (0,f.Operation.StopMotion,f.DispatchState.NotDispatched,None),
        (0,f.Operation.ServoMoveEnd,f.DispatchState.NotDispatched,None),
        (1,f.Operation.StopMotion,f.DispatchState.NotDispatched,None),
        (1,f.Operation.ServoMoveEnd,f.DispatchState.NotDispatched,None)]


def test_D06_worker_exit_hang_after_destructor_ack_keeps_both_phase_facts(rig,monkeypatch):
    monkeypatch.setenv('LPF_TEST_BLOCK','process_exit');make,d=rig;s=make(rpc_timeout_ms=150)
    result=s.close()
    assert result.sdk_destructors_completed and all(x.ok for x in result.stop.operations)
    assert result.process.reaped and result.process.forced and not result.ok
    assert 'normal worker exit deadline' in result.reason
    assert entered(d,'destructor_complete',0) and entered(d,'destructor_complete',1)
    assert entered(d,'process_exit_enter',-1)
    assert_reaped(s)


def test_D01_invalid_reply_phase_retains_first_reply_and_marks_second_unknown(rig,monkeypatch):
    monkeypatch.setenv('LPF_TEST_CORRUPT_REPLY','ready');make,d=rig;s=make();activate(s)
    pair=s.servo_joints([0.]*14,1)
    assert pair.arms[0].state==f.DispatchState.SdkReplied and pair.arms[0].reply.code==0
    assert pair.arms[1].state==f.DispatchState.Unknown and pair.arms[1].reply is None
    assert not pair.complete
    assert_reaped(s)
