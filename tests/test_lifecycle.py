import os
import time
import pytest
import libpyfairino as f

def reaped(session):
    assert session.last_worker_pid > 0
    with pytest.raises(ChildProcessError):
        os.waitpid(session.last_worker_pid, os.WNOHANG)

@pytest.mark.parametrize("mode", ["exit0", "exit1", "hang_ctor"])
def test_handshake_is_required(config, fake_backend, monkeypatch, mode):
    monkeypatch.setenv("LIBPYFAIRINO_TEST_MODE", mode)
    s = f.Session(config, backend_override=fake_backend)
    begin = time.monotonic()
    with pytest.raises(RuntimeError):
        s.connect(deny_network=True)
    assert time.monotonic()-begin < 3
    assert not s.connected
    reaped(s)
    s.close()

def test_unexpected_exit_after_ready(config, fake_backend, monkeypatch):
    monkeypatch.setenv("LIBPYFAIRINO_TEST_MODE", "exit_on_read")
    s = f.Session(config, backend_override=fake_backend)
    s.connect(deny_network=True)
    state=s.read_state()
    assert not state.valid and state.arms[0].state==f.DispatchState.Unknown
    reaped(s)
    assert not s.connected

@pytest.mark.parametrize("mode", ["hang_servo", "hang_stop"])
def test_operation_and_close_deadlines_reap(config, fake_backend, monkeypatch, mode):
    monkeypatch.setenv("LIBPYFAIRINO_TEST_MODE", mode)
    s=f.Session(config, backend_override=fake_backend)
    s.connect(deny_network=True)
    begin=time.monotonic()
    if mode=="hang_servo":
        s.left.ServoMoveStart()
        result=s.left.ServoJ([0.]*7,1)
        assert result.state==f.DispatchState.Unknown and "deadline" in result.reason
    else:
        cleanup=s.close()
        assert not cleanup.ok and cleanup.process.reaped
    assert time.monotonic()-begin < 3
    assert not s.connected
    reaped(s)

@pytest.mark.skipif(os.environ.get("LIBPYFAIRINO_REAL_SDK_DENIED_TEST")!="1",reason="real SDK constructor not exercised in this fake-only repair stage; previous denied trace retained")
def test_real_sdk_connect_is_network_denied(config):
    # Native seccomp is installed before dlopen and before any vendor constructor.
    s=f.Session(config)
    with pytest.raises(RuntimeError):
        s.connect(deny_network=True)
    assert not s.connected
    reaped(s)


def test_deny_network_filter_blocks_before_backend_construction(config, fake_backend, monkeypatch):
    monkeypatch.setenv("LIBPYFAIRINO_TEST_MODE", "probe_filter")
    s=f.Session(config, backend_override=fake_backend)
    s.connect(deny_network=True)
    assert s.connected
    s.close()
    reaped(s)


def test_timeout_does_not_signal_unrelated_child(config, fake_backend, monkeypatch):
    import subprocess
    other=subprocess.Popen(["/bin/sleep","10"])
    try:
        monkeypatch.setenv("LIBPYFAIRINO_TEST_MODE","hang_ctor")
        s=f.Session(config,backend_override=fake_backend)
        with pytest.raises(RuntimeError):s.connect(deny_network=True)
        reaped(s)
        assert other.poll() is None
    finally:
        other.terminate();other.wait(timeout=2)
