# Native ownership and lifecycle

Single-agent design comparison: in-process SDK construction cannot contain C exit; fork-only inside an existing Python process inherits unsafe runtime state. A fresh posix_spawn C++ worker with bounded AF_UNIX request/reply IPC is selected. Only this worker dlopens the real SDK backend, so ordinary imports and offline Model objects load no robot transport. An explicit connect constructs two SDK arms and requires a versioned ready handshake; exit code alone never proves success.

The parent C++ Session owns one process/socket and serializes SDK transactions. Timeouts fault and reap the child. Close requests stop/end before bounded teardown; killing a blocked SDK is not a verified physical emergency stop. A Linux seccomp deny-network policy is available before dlopen for real-constructor failure tests. No hidden vendor headers or guessed protocols are used.

Official SingleRobot declarations are adapted to value buffers and replies in the worker. C++ copies ServoJ inputs because the SDK mutates degrees to radians. Explicit confirmed configuration selects a real ART3 SDK enum without claiming it identifies ART7; configuration evidence, per-arm permutation, signs, offsets, limits and native loop speed caps are required. High-level hardware targets have exactly fourteen model radians. Active-torso nineteen-variable Models remain offline.

The native controller owns its sending thread, latest target, ramp and command watchdog; Python only submits intent. Per-arm calls are ordered, not atomically synchronized. q snapshots retain separate host intervals and unknown hardware freshness. Real offline IK/FK use the vendored fairino_ik C++ solver with explicit Rz(+90 degree) world base; vendor pure DH math remains separately available.
