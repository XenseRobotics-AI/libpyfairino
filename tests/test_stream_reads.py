import libpyfairino as f

def test_feedback_remains_available_during_slow_paired_sends(config, fake_backend):
    session=f.Session(config,backend_override=fake_backend)
    session.connect(deny_network=True)
    controller=f.Controller(session,[0.]*14,period_ms=5,command_timeout_ms=200)
    try:
        controller.start()
        for _ in range(12):
            state=session.read_state()
            assert state.valid, [(r.state, r.reason) for r in state.arms]
            controller.submit_joints([0.]*14)
        assert controller.status.running
    finally:
        controller.stop()
        session.close()
