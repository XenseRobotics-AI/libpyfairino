# libpyfairino: native dual-arm phase

Local Linux x86_64 / Python 3.12 package for two seven-axis arms. Python submits intent; the vendored fairino_ik C++ solver computes FK/IK and the C++ controller owns its ramp, sending thread and command watchdog. A fresh C++ worker owns both real SDK arm objects and serializes every official call. Importing the package, creating Kinematics, HardwareConfig or Session never connects hardware. Only explicit Session.connect starts the SDK process.

This implements the accepted dual-arm subset, not all 99 official declarations. The complete versioned [support matrix](docs/support-matrix.json) has supported, deferred and upstream-incompatible entries with original signatures and hashes. Thirty declarations are selected: SingleRobot's 26 named methods plus construction/destruction through the isolated lifecycle, and the two official explicit-DH mathematical functions. Two RobotTool upgrade members have no matching definitions. Ten JNTS-related declarations use incompatible header/library types. Neither is callable here. Waist/head/chassis, upgrades, ROS and LeRobot integration are deferred. No vendor implementation or hidden protocol was guessed.

## Build and install locally

Use a clean Python 3.12 venv. The pinned build dependencies select Pinocchio 3.9 and eiquadprog 1.3.0 with compatible Boost 1.89. eiquadprog 1.2.9 conflicts with that Boost version. Eigen 3.4 headers and a C++17 compiler are required; the isolated cmeel prefix also supplies Eigen when available. No Qt or MeshCat is needed.

```sh
python3.12 -m venv /tmp/fairino-build
/tmp/fairino-build/bin/pip install build 'scikit-build-core==0.11.6' 'pybind11==3.0.1' 'pin==3.9.0' 'cmeel-urdfdom==4.0.1' 'cmeel-tinyxml2==10.0.0' 'eiquadprog==1.3.0'
CMAKE_BUILD_PARALLEL_LEVEL=1 /tmp/fairino-build/bin/python -m build --wheel --no-isolation -Cbuild-dir=/tmp/fairino-cmake --outdir /tmp/fairino-dist .
/tmp/fairino-build/bin/pip install /tmp/fairino-dist/libpyfairino-*.whl
```

The wheel carries real SDK libraries with their required loader names, the native worker/backend and model resources. Its RPATH uses package-relative paths and the pinned cmeel runtime. This is a local platform wheel, not a manylinux portability certification. See third_party/provenance.json for exact original bytes and source commit. Never publish, push or upload the private vendor files or a wheel containing them. No upstream license text was invented: no client SDK LICENSE or fairino_ik LICENSE file was present in the supplied source; the service package's Apache metadata does not license the client.

## Offline FK and IK

```python
import math
from libpyfairino import Kinematics, Options
model = Kinematics()
options = Options()
q = [math.radians(x) for x in [-30,45,0,60,0,0,0,30,45,0,60,0,0,0]]
left = model.forward_kinematics(q, "left", options)
right = model.forward_kinematics(q, "right", options)
result = model.solve_ik(left, right, q, options)
print(result.success, result.q)
```

Model q is exactly [left 7, right 7] radians, or 19 variables with active_torso=True. The extra torso order is j20,j19,j18,j16,j15; head and wheels stay locked. In arm-only mode Options.torso_position and head_position determine the fixed body configuration and are used in every FK/IK call. Full nq32 uses continuous-wheel cos/sin coordinates and must never be sliced into hardware targets.

Poses are flat, row-major sixteen-element transforms, metres and radians. Kinematics explicitly defaults to Rz(+90 degree), x forward/y left/z up; pass URDF_BASE_POSE for raw URDF base axes. Low-level Model without the facade follows upstream identity-base behavior. Wrist-link frames are default; left_flange/right_flange add the upstream estimated -0.114 m x offset, which is not a calibrated TCP. The URDF has nominal geometry, asymmetric elbow limits, zero source velocity limits and sampled body-box avoidance; it is not a physical safety certificate.

The separate sdk_math module binds official DHParam, DHCMPSTPARAM, JOINTLIMIT, forwardKinematic and inverseKinematic. DH lengths are metres and DH angles radians, joint arrays degrees, and homogeneous translations millimetres. MathResult.code preserves the C return value and output_finite separately reports whether the relevant outputs are finite. The library can return zero with NaN outputs for unsupported or degenerate synthetic DH geometry; zero alone is not solution validity or physical model validation. all_solutions_deg is meaningful only where the library wrote it; unwritten entries remain NaN.

## Explicit hardware configuration and lifecycle

No ART7 enum is present in this delivery. HardwareConfig requires an actual SDKRobotType enum and nonempty model_compatibility_evidence and joint_mapping_evidence. Do not select an ART3 entry merely because the machine is ART7-R7. A vendor-confirmed choice, per-arm SDK-to-model permutation, signs, offsets, confirmed SDK-degree limits and model-radian velocity caps must be supplied explicitly. No usable machine-specific configuration is bundled. Hardware Cartesian submission additionally requires fixed_torso_rad (five explicit model radians), fixed_head_rad and cartesian_frame_evidence together. That evidence must identify the chosen Model base, wrist/flange/TCP transform and physical fixed-body calibration. The supplied Options must match those fixed values; defaults cannot silently substitute for them. Joint-only sessions do not require this Cartesian-specific configuration.

For SDK joint j of arm a, the native conversion is sdk_deg[a*7+j] = sign[a*7+j] * model_rad[a*7+permutation[a*7+j]] * 180/pi + offset_deg[a*7+j]. Each arm permutation is a bijection of 0..6, never a full_q slice. DH compensation has the official 28-value constructor shape and does not replace all nominal robot geometry.

Session(config) is offline. connect() explicitly starts the real SDK worker at the vendor's fixed address 192.168.58.1:8081; the public SDK cannot select another address. There is no constructor call in the Python host. Connect succeeds only after both arm objects and confirmed soft-limit setup complete a versioned ready handshake. exit(0), exit(1), EOF and a handshake timeout are all failures. A launch gate prevents backend loading until the parent holds a Linux pidfd. Every transaction has an RPC deadline; failure closes IPC, signals that pidfd and waitpid-reaps that exact worker, avoiding signals to a reused numeric PID. Linux pidfd support is required. close requests StopMotion and ServoMoveEnd for both arms, then bounds teardown and reaps the worker. Killing a stuck worker does not prove the robot physically stopped. close can raise if stop/end could not be acknowledged. Logs remain in the session's temporary log_directory for diagnosis.

Session(..., backend_override=...) is an explicit developer test seam, not the real SDK by default. connect(deny_network=True) installs a Linux x86_64 seccomp network-denial policy before backend dlopen and SDK constructors. It uses inherited AF_UNIX read/write IPC and forbids network socket creation, connection, send/receive message syscalls, and io_uring setup. If the filter cannot be installed the worker fails, rather than attempting a connection. Do not use this diagnostic mode as a production connection.

## Dual-arm control and observations

Session.left and right expose the selected official CamelCase methods. Values are SDK degrees or mm/deg exactly as declared. Every call returns SdkReply with original integer code, bounded error_bytes, UTF-8 replacement-decoded error, output values and the relevant state/arm_angle_deg. Check ok before using outputs. GetForwardKin returns six pose values and arm_angle_deg; GetJointSoftLimitDeg returns [negative 7, positive 7]. ServoCart accepts 1..128 seven-value points per request. Axis IDs are 0..7 and tool IDs 1..19. ServoJ adapter periods are 1..1000 ms. Runtime soft-limit setters cannot expand confirmed bounds. Factory calibration functions are exposed but never called by import/start or offline tests.

Session.servo_joints accepts exactly fourteen model radians, converts each once, copies per-arm SDK degree buffers, and calls the two arms in order. The SDK itself converts its input in place to radians; Python memory is never passed directly. An unsent second arm is explicitly an error, not fabricated success. No atomic simultaneous execution is promised. All SDK entry calls run in one worker loop, avoiding concurrent entry from Python threads; this does not prove the vendor's own internal background threads are correct. Multiple live real sessions/processes against the same controller are unsupported and must not be used without a vendor contract.

Controller(session, initial_model_q_rad, period_ms=5, command_timeout_ms=500) owns the native control loop. Explicitly connect first; start switches servo mode but does not enable axes. The initial anchor must match the physically checked current pose. start reads SDK q and rejects an anchor mismatch above the adapter tolerance of 0.01 rad, then anchors the ramp to the observed value; cached SDK q is still not proof of hardware freshness. submit_joints validates fourteen finite model radians and limits. Native velocity caps ramp toward the latest target; a command timeout triggers stop/end. submit_cartesian runs real C++ IK and submits only successful fourteen-variable results; a 19-variable active-torso model is rejected for hardware commands. sent_sequence identifies the target associated with the last interpolated command, not target arrival or device execution. stop joins the native thread and records unconfirmed stop errors in status. The nominal period includes vendor per-arm sleeps and IPC; it is not a measured or guaranteed rate.

read_state returns fourteen model radians and separate per-arm host_read_begin/end_ns, raw replies and numeric validity. hardware_freshness_known is always False and hardware_timestamp_ns is None. It does not fabricate dq, torque, device timestamps, same-frame pairing or execution confirmation. Basic record need not require enhanced fields; a later LeRobot adapter must select and validate its own observation/action pairing and data-quality contract.

## Verification

```sh
python tools/check_supported_api.py --output /tmp/fairino-supported.json
python -m pytest -q tests
```

Tests link the real selected SDK declarations and use the real official math and vendored C++ IK. Lifecycle/control behavior also uses an explicitly labeled fake SDK compiled only under pytest into /tmp, never installed in the wheel. The real SDK failure test uses deny_network=True before any vendor construction. No test executes a vendor motion example, connects a device, starts ROS or upgrades firmware. See the implementation report for relocated-wheel and syscall evidence.
