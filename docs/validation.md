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
