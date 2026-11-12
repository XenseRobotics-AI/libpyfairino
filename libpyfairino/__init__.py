"""Native dual-arm APIs. Import and Kinematics never connect hardware."""
from pathlib import Path
from ._native import (Model, Options, Result, SDKRobotType, SDKRobotName, ARMErrorCode,
    HardwareConfig, Session as _Session, Arm, SdkReply, Snapshot, Controller, ControllerStatus, sdk_math, CallResult, PairResult, StopReport, ReapReport,
    CleanupReport, FaultRecord, DispatchState, FaultOrigin, Operation)

__version__ = "0.2.3"

class CleanupError(RuntimeError):
    """A context exited without an earlier exception, but cleanup was not fully confirmed."""
    def __init__(self, result):
        super().__init__("SDK cleanup failed or remains unconfirmed; inspect .result")
        self.result = result

class Session(_Session):
    """Context orchestration around the native structured cleanup contract."""
    def __enter__(self):
        if not self.connected:
            raise RuntimeError("call connect explicitly before entering a session context")
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            result = self.close()
        except BaseException as cleanup_error:
            if exc is None:
                raise
            try:
                exc.add_note("SDK cleanup raised: " + str(cleanup_error))
            except BaseException:
                pass
            return False
        if not result.ok:
            if exc is None:
                raise CleanupError(result)
            try:
                exc.add_note("SDK cleanup was not confirmed; inspect session.last_cleanup")
            except BaseException:
                pass
        return False

DEFAULT_BASE_POSE = (0., -1., 0., 0., 1., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.)
URDF_BASE_POSE = (1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.)

def model_path():
    return str(Path(__file__).resolve().parent / "assets/fairino/fairino_wheel_legged_robot.urdf")

def Kinematics(*, active_torso=False, left_frame="left_waist2_Link",
               right_frame="right_waist2_Link", base_pose=DEFAULT_BASE_POSE, urdf_path=None):
    """Create the real native Model with an explicit world base and packaged URDF.

    Flat poses are row-major, translations metres, joints radians. A 19-variable
    model is offline only; Controller.submit_cartesian accepts a 14-variable model.
    Extra left_flange/right_flange frames use the upstream estimated -0.114 m offset.
    """
    return Model(urdf_path or model_path(), left_frame, right_frame, base_pose, active_torso)
