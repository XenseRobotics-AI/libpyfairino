"""FA-01: semantic Ready success versus real adapter failure and protocol injection."""
import os
import pytest
import libpyfairino as f

def reaped(session):
    assert not session.connected
    assert session.last_cleanup.process.reaped
    assert session.diagnostic
    with pytest.raises(ChildProcessError):os.waitpid(session.last_worker_pid,os.WNOHANG)

def session(config,fake_backend):
    return f.Session(config,backend_override=fake_backend)

def test_FA01_failed_setup_cannot_be_promoted_by_ready_tag(config,fake_backend,monkeypatch):
    monkeypatch.setenv('LPF_TEST_INIT_LIMIT_CODE','-2')
    monkeypatch.setenv('LPF_TEST_INIT_FRAME','failure_to_ready')
    s=session(config,fake_backend)
    try:
        with pytest.raises(RuntimeError,match='-2'):s.connect(deny_network=True)
        reaped(s)
    finally:s.close()

def test_FA01_original_failure_reply_keeps_original_sdk_code(config,fake_backend,monkeypatch):
    monkeypatch.setenv('LPF_TEST_INIT_LIMIT_CODE','-2')
    s=session(config,fake_backend)
    with pytest.raises(RuntimeError,match='code -2'):s.connect(deny_network=True)
    reaped(s)

@pytest.mark.parametrize('field',['code','sdk_flag','count','error','error_tail','fault_id','fault_code','fault_flag','sequence','operation','arm','version','stage','short','exit0'])
def test_FA01_single_anomalous_ready_field_fails_closed(config,fake_backend,monkeypatch,field):
    monkeypatch.setenv('LPF_TEST_INIT_FRAME',field)
    s=session(config,fake_backend)
    try:
        with pytest.raises(RuntimeError):s.connect(deny_network=True)
        reaped(s)
    finally:s.close()

@pytest.mark.parametrize('variant',['normal','unused'])
def test_FA01_normal_ready_unused_fields_and_reconnect(config,fake_backend,monkeypatch,variant):
    if variant=='unused':monkeypatch.setenv('LPF_TEST_INIT_FRAME','unused')
    s=session(config,fake_backend)
    for attempt in range(2):
        s.connect(deny_network=True)
        assert s.connected and not s.diagnostic and not s.poll_status()
        assert s.left.GetActualJointPosDegree().ok
        result=s.close()
        assert result.ok and result.process.reaped and result.sdk_destructors_completed
        assert not result.process.forced
