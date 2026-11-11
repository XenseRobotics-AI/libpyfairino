# Native ownership after D-01..D-06

A fresh posix_spawn C++ worker owns both SDK arms. Host Model and ordinary construction remain offline. A pidfd launch gate precedes backend loading; explicit denied tests install seccomp before dlopen. Every SDK entry is made through real public declarations.

One non-recursive Session mutex owns active limits, complete pair preparation and ordered dispatch, single calls, and lifecycle transactions. Internal prepare_locked/dispatch_locked do not reenter the public locking methods. Host fault cache has a separate short-lived mutex. Controller target/status locks never cover SDK waiting. Stop is an atomic request before lifecycle join.

The Controller captures a complete target/sequence/submitted/deadline value. Existing old transactions may complete only inside that old lease. Per-arm pre-dispatch checks and min(target deadline,RPC deadline) enforce that policy; successful pair bookkeeping is updated only after both original SDK-zero replies.

CallResult and PairResult preserve the boundary between local omission, native rejection, original SDK response, and uncertainty. Worker stop/watchdog failures are first-failure records, not success flags. PollStatus and ordinary replies propagate them; reconstruction is the only reset, with the previous cleanup snapshot preserved.

StopAll/Shutdown send each stop/end progress result before subsequent potentially blocking work. Shutdown then explicitly destroys both arms, acknowledges that phase, and exits. Parent normally waits/reaps; deadline or communication failure falls back to checked pidfd termination and nonblocking/polled reap. Kernel/scheduling failures can remain unknown and are reported rather than hidden by an infinite wait.

Hardware Cartesian uses an Options value copied before GIL release, exact Model-derived finite closed intervals, exact calibration matching, and expanded fixed-coordinate checks. Vendored clamping remains untouched for offline callers. The caller exclusively owns each hardware Model during submission; arbitrary coupling/infinite-limit models are outside the supported contract.
