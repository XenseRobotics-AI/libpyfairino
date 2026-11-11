# Dual-arm offline validation snapshot

2026-11-11, Linux x86_64, Python 3.12.13, GCC 13.3.0. No real device connection or motion. This is the phase-one dual-arm subset, not full SDK or hardware acceptance.

Wheel: `libpyfairino-0.1.0-cp312-cp312-linux_x86_64.whl`. SHA-256 `444d11452c74f255b7d2acfa8910b2e44c9b5b7438e7d91fe226690fc6c67710`.

## Reproducible checks

The build ran in `/tmp/libpyfairino-build`, a new venv without system site packages, with pinned dependencies from pyproject.toml. PEP 517 `--no-isolation` used that already isolated, provisioned build environment. The automatic fresh PEP 517 dependency-install path was not separately exercised.

```sh
CMAKE_BUILD_PARALLEL_LEVEL=1 /tmp/libpyfairino-build/bin/python -m build --wheel --no-isolation -Cbuild-dir=/tmp/libpyfairino-cmake --outdir /tmp/libpyfairino-dist .
/tmp/libpyfairino-build/bin/python tools/check_supported_api.py --output /tmp/supported.json
/tmp/libpyfairino-build/bin/python tools/verify_sources.py
/tmp/libpyfairino-build/bin/python -B -m pytest -q -p no:cacheprovider tests
```

Build and real SDK selected-declaration link exit 0. The support matrix has 30 selected declarations, 57 deferred and 12 upstream-incompatible, totaling the original 99 declaration entries. The 30 selected comprise all 28 explicit SingleRobot declarations (including lifecycle) plus two official pure mathematical functions; they are not 30 independent runtime functions or proof of hardware behavior.

21 pytest cases passed. Tests separately cover the real C++ IK/FK and official C mathematical library, configuration/shape/units/errors, and clearly labeled test-double lifecycle/control behavior. Test-double cases verify exit(0), exit(1), pre-handshake exit, connect/call/close timeouts, pidfd-scoped reaping without signaling an unrelated child, ordered arm routing, copied ServoJ buffers, bounded 1024-byte errors, actual fake-backend watchdog behavior, startup-anchor validation, fixed-body Cartesian configuration and native ramp/watchdog. The fake SDK is compiled into /tmp by pytest and is not installed in the wheel. No fake result substitutes for the real SDK compile/link checks.

## Installed artifact and source independence

The pinned dependency environment was copied to `/tmp/libpyfairino-relocated`; the final wheel was then force-reinstalled there with pip `--no-deps`. This avoids repeated downloads and is explicitly not a second clean dependency-resolution experiment. Its interpreter ran a copied verifier under `python -I`, with PYTHONPATH, LD_LIBRARY_PATH, LD_PRELOAD and AMENT_PREFIX_PATH removed, from `/tmp/libpyfairino-unrelated-cwd`.

Installed package: `/tmp/libpyfairino-relocated/lib/python3.12/site-packages/libpyfairino`. Packaged model: `/tmp/libpyfairino-relocated/lib/python3.12/site-packages/libpyfairino/assets/fairino/fairino_wheel_legged_robot.urdf`. Real native FK-to-IK succeeded for two reachable hand targets, with position residuals `3.4963988615301534e-09` and `3.4963859371067773e-09` metres in that numerical case. This is a numerical residual, not measured robot accuracy or performance. Real library and model hashes matched the provenance. ELF RPATH/NEEDED for the installed native components contained no /tmp or /home build/source paths. Process mappings resolved the installed cmeel runtime and installed kine library, with no source SDK library, main transport SDK or backend loaded by offline import/model construction.

## Real SDK failure and no-network evidence

A separate `strace -f -e trace=network` offline import/model probe exited 0 with zero network system calls. A real SDK construction-failure probe used Session.connect(deny_network=True), so PR_SET_NO_NEW_PRIVS and PR_SET_SECCOMP completed before dlopen opened the real installed backend. The trace then observed `socket(AF_INET, SOCK_STREAM, IPPROTO_IP) = -1 EPERM`. There was no successful INET socket or robot connection. Connect raised for missing ready handshake, and the exact pidfd-owned worker was waitpid-reaped. A separate test double checked both socket and connect are denied before its constructor proceeds.

Detailed commands and logs are local at `/tmp/libpyfairino-coordination/dual-arm-check-commands.json`, `dual-arm-runtime-probes.json`, `dual-arm-network-denied.strace`, `dual-arm-offline.strace`, and `dual-arm-build.log`.

## Acceptance limits

No successful real SDK connection, enable, motion, stop confirmation, concurrent SDK behavior, reconnect after actual network loss, physical calibration, control-rate guarantee or real training-data quality is claimed. The original upstream missing members and JNTS incompatibility remain outside the dual-arm subset. The official C inverse function returned zero with NaN q for the synthetic DH case used in a binding-contract check; original code and output_finite=False are both preserved. The main FK/IK path uses the verified vendored C++ solver instead. ART7 enum compatibility, live mapping, limits, base/TCP and fixed-body calibration require explicit, truthful user/vendor configuration before hardware use.
