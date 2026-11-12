# Pinocchio 4.1 runtime alignment (0.2.2)

2026-11-12. Build and runtime now use Pinocchio 4.1.0, Boost 1.90.0,
NumPy 2.2.4, urdfdom 6.0.0 and tinyxml2 11.0.0. Build-only QP headers
come from eiquadprog 1.3.2. The previous 1.3.0 distribution requires Boost
1.89 and cannot resolve with this combination. URDF build headers are
requested through `cmeel-urdfdom[build]`.

The copied IK source matches the `feat/pinocchio-4.1.0` worktree in
`fairino_ik`. Provenance `ik_commit` pins the committed source revision,
`ik_worktree_modified` is false, and per-file SHA-256 values identify the
exact copied bytes. Official SDK files are unchanged.
`tools/verify_sources.py` verifies both copies and reports this distinction.

Both libraries now use Pinocchio's exported CMake targets. The IK headers
use Pinocchio 4 public multibody/spatial headers. No IK equations, mappings,
control logic, watchdogs or hardware configuration gates were changed.

A fresh Python 3.12 virtual environment resolved and installed these exact
requirements. Both the explicit build environment and the default PEP 517
isolated build completed; the final isolated-build wheels were reinstalled
and passed the same tests and co-loading checks. Local wheels are retained
in each repository's ignored `dist/` directory. After installing the built wheels, `pip check` passed,
`fairino_ik` had 50 passing tests including offscreen Qt, and libpyfairino
had 138 passing tests with one real-SDK constructor test skipped. Fake SDK
tests use `deny_network=True`; no real controller was connected.

`tools/verify_installed.py` passed with PYTHONPATH, LD_LIBRARY_PATH and
LD_PRELOAD removed. It checked installed FK/IK, model and SDK asset hashes,
relative ELF runtime paths, and absence of the transport SDK on offline import.

The existing Tianji API 9 binary from the LeRobot installation was loaded
with the new Fairino wheel in each import order. Both libraries passed
FK/IK and mapped the same single Pinocchio 4.1 shared library. Reproduce
in an environment containing both packages:

```sh
python -m pip check
python -I -B -m pytest -q -p no:cacheprovider tests
python -I -B tools/verify_installed.py --output /tmp/fairino-installed.json
python -I -B tools/verify_shared_runtime.py
```

For an existing source installation of Tianji, pass
`--tianji-source /path/to/libpytianji` to the last command. The script only
constructs offline kinematics models, never Session or control objects.
Local evidence is under `/tmp/fairino-pin41-*.log` and
`/tmp/fairino-pin41-shared-runtime.json`. The existing LeRobot conda
environment was not modified. This validates native software compatibility,
not robot calibration, physical motion or the pending LeRobot adapter.

---

# Historical FA-01 handshake patch validation (0.2.1)

2026-11-12. Baseline 81ecac80a514ced445e3e57272f061ede3fa49dc; D-01..D-06 remain accepted within their existing scope. Only Initialize Ready semantic validation, diagnostics and patch-version documentation changed.

Wheel `libpyfairino-0.2.1-cp312-cp312-linux_x86_64.whl` SHA-256 `2a0984f6ac49a48c96af4ee44ed7f5dadb8705f721aba3f1b0e9108bb06b8ce6`. The existing 0.2.0 installed wheel actually failed the new FA-01 regression: a failed setup Reply changed only to Ready did not raise. The failure is recorded at /tmp/libpyfairino-coordination/dual-arm-handshake-red.log.

The patch requires existing fixed-size/version/request identity validation plus Ready code=0, has_sdk_reply=0, count=0, an empty full error buffer and no fault id/code/SDK-reply data. It checks before trusting the fault ledger, with diagnostic field values, connected=False and existing exact-worker cleanup facts on failure. Args/values and other unused command-specific data may retain the initialization request; they are not required to be zero and no C++ padding is compared. Normal failure Reply keeps its -2 SDK code.

19 targeted cases passed: normal Ready and reconnect; unused-field nonzero/NaN control; unmodified production failure Reply; multi-field failed Reply changed to Ready; independent code, SDK flag, count, first/tail error, fault id/code/flag; wrong identity/version/stage/length; and exit(0)/EOF. All active sessions use an explicit fake backend plus deny_network=True.

Complete applicable suite: 138 passed, 1 skipped. The skip is still the real SDK constructor test, not executed. No vendor constructor or hardware path was touched. Build/install and relocated wheel checks, real 30-declaration compile/link, 50 source hashes and pip check all exited 0. Runtime/dependencies were provisioned/copied from the existing pinned isolated environment, not freshly resolved on another machine. Command records and logs are in /tmp/libpyfairino-coordination/dual-arm-handshake-command-results.json and dual-arm-handshake-*.log/xml/json.

A consistent success message cannot prove a buggy/malicious worker told the truth; this patch rejects contradictions, not hardware state. Original physical calibration, stop/execution, concurrency and platform limits remain. No full-99, platform or additional feature scope was added.

## Historical D-01..D-06 validation record (0.2.0)

# D-01..D-06 repair validation

2026-11-12 local date. Baseline main commit e8f8373c3f025c0c02412e0b27e3a7855143cf1f. New result API is 0.2.0; see api-migration.md. This is offline software evidence, never a hardware motion/stop acceptance.

Wheel `libpyfairino-0.2.0-cp312-cp312-linux_x86_64.whl` SHA-256 `dd1fe787286e7e1e44b4eaf34bcf7ca947f8834956813fcd6909b01bd4e3030c`. Built with GCC 13.3.0, Python 3.12 and the existing isolated pinned Pinocchio 3.9/eiquadprog 1.3 build environment.

## Before/after evidence

The preserved 0.1 frozen wheel was run with an explicit test SDK and deny_network=True. `/tmp/libpyfairino-fix-baseline/reproduce.py` observed each old violation: left fake q reached 20 degrees before the local right-arm rejection; 100-rad fixed torso was submitted after Model clamp; the right SDK call occurred after an expired old target; successful getter hid watchdog stop failures; close returned None for nonzero stop codes; no delayed destructor completed in that run. The assertions intentionally confirm old violations; exit 0 does not mean old acceptance passed.

A separate new malformed-phase test actually failed before the corresponding protocol fix: a Ready frame for a ServoJ was wrongly accepted. The same case passes in the final suite, preserving the left original reply and marking the right Unknown. Logs: dual-arm-fix-baseline.log and dual-arm-fix-invalid-phase-red.log in /tmp/libpyfairino-coordination.

## Current regression command and result

```sh
/tmp/libpyfairino-fix-env/bin/python -I -B -m pytest -q -p no:cacheprovider tests --basetemp /tmp/libpyfairino-fix-final-suite
```

119 passed, 1 skipped. The skip is the real SDK constructor test, not executed in this fake-only repair stage. No device path was contacted. New D regressions total 99: D01 9, D02 62, D03 4, D04 8, D05 13, D06 3. Existing active tests total 20. Four existing kinematics tests run the real C++ solver and official mathematical library; D02 combines real Models with explicitly fake hardware sessions. Other control/lifecycle tests use the real adapter/backend code with a test SDK. No fake backend or IPC fault injector is installed in the wheel.

The regressions use event barriers, not a claim that repeating timing loops proves concurrency safety. They cover whole-pair active-limit validation and setter serialization; both raw nonzero positions, lost/invalid replies and preserved known/unknown facts; six actual Model bounds with exact endpoints and nextafter inside/outside, nonfinite/mismatched values, a changed-limit Model and post-copy Options mutation; old target deadline/stop/new submission interleavings; first stop faults surviving getters/retries; all stop/end combinations, context exceptions, each destructor phase, early exit and post-ack worker-exit hang. The source contracts still exclude concurrent use of the same hardware Model and unsupported custom mimic/infinite-limit semantics.

## Artifact and real-library checks

`tools/check_supported_api.py` recompiled/linked all selected 30 real official declarations, exit 0. `tools/verify_sources.py` confirmed 50 vendored/source file hashes and unchanged IK HEAD, exit 0. No factory header, library, macro or vendored solver was edited. Support matrix remains 30/57/12 of the 99 declaration entries.

The final wheel was force-installed into /tmp/libpyfairino-fix-env and /tmp/libpyfairino-fix-relocated, using the already provisioned copied dependency runtime. This is not a new clean dependency-resolution experiment. A copied installed-artifact verifier ran under python -I from an unrelated /tmp cwd with PYTHONPATH/LD_LIBRARY_PATH/LD_PRELOAD/AMENT_PREFIX_PATH removed. Model/resources/vendor hashes and relative RPATH passed; actual FK-to-IK passed, with no source SDK mappings or transport SDK loaded offline. pip check passed.

Raw fake lifecycle strace: normal close received [0,0,0,0], completed both SDK destructors, exited 0 and reaped without any pidfd_send_signal syscall. Hanging destructor retained [-2,-3,-4,-5], marked destruction unconfirmed, used a single pidfd SIGKILL fallback and reaped signal 9. No INET socket was created in those probes. The additional regression separates acknowledged SDK destruction from an unconfirmed later process exit.

Commands and logs: /tmp/libpyfairino-coordination/dual-arm-fix-command-results.json, dual-arm-fix-pytest.xml/log, dual-arm-fix-build.log, dual-arm-fix-normal.strace, dual-arm-fix-hang.strace, dual-arm-fix-relocated.json. The full D-ID change report is dual-arm-fix-report.md in that directory.

## Limits

No real SDK constructor, connection, motion, enable, physical stop, network-loss recovery or real rate was exercised in this repair run. Earlier denied-constructor evidence remains historical. Model residuals are numerical quantities, not physical accuracy. SDK zero, destructor acknowledgement and process exit/reap do not confirm physical stopping. Queueing, scheduling, syscall failure or uninterruptible kernel behavior are not covered by a hard real-time promise; unconfirmed stages and reap failures remain explicit. ART7 enum/mapping/limits/DH/base/TCP and fixed-body calibration still need truthful external confirmation.
