import math
import time
import pytest
import libpyfairino as f

def test_official_adapter_values_and_mutation(config, fake_backend):
    s=f.Session(config, backend_override=fake_backend)
    assert not s.connected
    s.connect(deny_network=True)
    for arm in (s.left,s.right):
        assert arm.ServoMoveStart().ok
    q=[10.]*7
    assert s.left.ServoJ(q,1).ok
    assert q==[10.]*7
    assert s.left.GetActualJointPosDegree().values==q
    assert s.right.GetActualJointPosDegree().values==[0.]*7
    assert s.left.GetForwardKin([1.,2.,3.,4.,5.,6.,7.]).arm_angle_deg==7
    assert s.left.GetInverseKin([1.,2.,3.,4.,5.,6.],7).values==[1.,2.,3.,4.,5.,6.,7.]
    assert s.left.GetJointSoftLimitDeg().values==[-180.]*7+[180.]*7
    assert s.left.GetRobotMotionDone().state==1
    state=s.read_state()
    assert state.valid and not state.hardware_freshness_known
    assert state.hardware_timestamp_ns is None
    assert state.q_rad[:7]==pytest.approx([math.radians(10.)]*7)
    assert state.host_read_begin_ns[0]<=state.host_read_end_ns[0]<=state.host_read_begin_ns[1]
    for bad in ([0.]*6,[0.]*8):
        with pytest.raises(TypeError):s.left.ServoJ(bad,1)
    for bad in ([float("nan")]+[0.]*6,[181.]+[0.]*6):
        with pytest.raises(ValueError):s.left.ServoJ(bad,1)
    with pytest.raises(ValueError):s.left.ServoJ([0.]*7,0)
    with pytest.raises(ValueError):s.left.AxisEnable(8)
    with pytest.raises(ValueError):s.left.SetSpeed(101.)
    with pytest.raises(ValueError):s.left.ServoCart([])
    with pytest.raises(TypeError):s.servo_joints([0.]*19,1)
    pid=s.last_worker_pid
    s.close();s.close()
    import os
    with pytest.raises(ChildProcessError):os.waitpid(pid,os.WNOHANG)

def test_error_buffer_is_bounded_and_unsent_arm_is_not_success(config, fake_backend, monkeypatch):
    monkeypatch.setenv("LIBPYFAIRINO_TEST_MODE","error_buffer")
    s=f.Session(config,backend_override=fake_backend);s.connect(deny_network=True)
    s.left.ServoMoveStart();s.right.ServoMoveStart()
    replies=s.servo_joints([0.]*14,1)
    assert replies[0].code==-2 and len(replies[0].error_bytes)==1024
    assert replies[0].error=="X"*1024
    assert not replies[1].ok and "not sent" in replies[1].error
    s.close()

def test_native_worker_watchdog(config, fake_backend):
    s=f.Session(config,backend_override=fake_backend);s.connect(deny_network=True)
    s.left.ServoMoveStart();time.sleep(.2)
    reply=s.left.ServoJ([0.]*7,1)
    assert not reply.ok and "watchdog" in reply.error
    s.close()

def test_native_controller_ramp_and_watchdog(config, fake_backend):
    s=f.Session(config,backend_override=fake_backend);s.connect(deny_network=True)
    c=f.Controller(s,[0.]*14,period_ms=1,command_timeout_ms=80)
    c.start();seq=c.submit_joints([.1]*14)
    deadline=time.monotonic()+1
    while c.status.sent_sequence!=seq and time.monotonic()<deadline:time.sleep(.002)
    status=c.status
    assert status.sent_sequence==seq and 0<status.commanded_q_rad[0]<.1
    assert status.host_send_end_ns>0
    deadline=time.monotonic()+2
    while c.status.running and time.monotonic()<deadline:time.sleep(.01)
    assert not c.status.running and "watchdog" in c.status.error
    c.stop();s.close()

def test_explicit_mapping_and_model_limits():
    config=f.HardwareConfig(f.SDKRobotType.ART3_R7,[-180.]*14,[180.]*14,list(reversed(range(7)))*2,[-1.]*14,[5.]*14,[1.]*14,"synthetic test","synthetic test")
    s=f.Session(config)
    q=[.1*j for j in range(7)]*2
    assert s.sdk_degrees(q)[:7]==pytest.approx([5.-math.degrees(x) for x in reversed(q[:7])])
    with pytest.raises(ValueError):
        f.HardwareConfig(f.SDKRobotType.ART3_R7,[-180.]*14,[180.]*14,[0]*14,[1.]*14,[0.]*14,[1.]*14,"test","test")
    with pytest.raises(ValueError):
        f.HardwareConfig(f.SDKRobotType.ART3_R7,[-180.]*14,[180.]*14,list(range(7))*2,[1.]*14,[0.]*14,[1.]*14,"","")


def test_controller_start_anchor_and_active_torso_are_checked(config, fake_backend):
    s=f.Session(config, backend_override=fake_backend);s.connect(deny_network=True)
    c=f.Controller(s,[.1]*14)
    with pytest.raises(ValueError,match="startup anchor"):
        c.start()
    c=f.Controller(s,[0.]*14)
    c.start()
    with pytest.raises(ValueError,match="fourteen"):
        c.submit_cartesian(f.Kinematics(active_torso=True), f.URDF_BASE_POSE, f.URDF_BASE_POSE, [0.]*19, f.Options())
    c.stop();s.close()


def test_runtime_limits_remain_consistent(config, fake_backend):
    s=f.Session(config,backend_override=fake_backend);s.connect(deny_network=True)
    assert s.left.SetLimitNegative([-90.]*7).ok
    with pytest.raises(ValueError,match="positive soft limit"):
        s.left.SetLimitPositive([-100.]*7)
    assert s.left.SetLimitPositive([90.]*7).ok
    s.left.ServoMoveStart()
    with pytest.raises(ValueError,match="active confirmed"):
        s.left.ServoJ([100.]*7,1)
    assert s.left.GetJointSoftLimitDeg().values==[-90.]*7+[90.]*7
    s.close()


def test_hardware_cartesian_requires_explicit_fixed_body_configuration(config, fake_backend):
    s=f.Session(config,backend_override=fake_backend);s.connect(deny_network=True)
    c=f.Controller(s,[0.]*14);c.start()
    m=f.Kinematics();options=f.Options();options.body_avoidance=False
    left=m.forward_kinematics([0.]*14,"left",options)
    right=m.forward_kinematics([0.]*14,"right",options)
    with pytest.raises(ValueError,match="explicit fixed torso"):
        c.submit_cartesian(m,left,right,[0.]*14,options)
    c.stop();s.close()
    calibrated=f.HardwareConfig(f.SDKRobotType.ART3_R7,[-180.]*14,[180.]*14,list(range(7))*2,[1.]*14,[0.]*14,[1.]*14,"synthetic SDK test","synthetic mapping",fixed_torso_rad=[0.]*5,fixed_head_rad=0.,cartesian_frame_evidence="SYNTHETIC MODEL ONLY: default world and wrist links")
    s=f.Session(calibrated,backend_override=fake_backend);s.connect(deny_network=True)
    c=f.Controller(s,[0.]*14);c.start()
    result=c.submit_cartesian(m,left,right,[0.]*14,options)
    assert result.success
    options.torso_position=[.1,0.,0.,0.,0.]
    with pytest.raises(ValueError,match="fixed torso differs"):
        c.submit_cartesian(m,left,right,[0.]*14,options)
    c.stop();s.close()
