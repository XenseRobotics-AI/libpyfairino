# Breaking API migration from 0.1 to 0.2

All repository callers were migrated; the old result shape is intentionally not retained as a compatibility alias.

| 0.1 call/result | 0.2 contract |
|---|---|
| Arm method returns SdkReply; transport throws | Arm method returns CallResult. Use state, reason and optional reply; raw fields are reply.code/error_bytes/values/arm_angle_deg/motion_state. Parameter validation still raises before dispatch. |
| servo_joints returns a two-element array | PairResult.arms contains each fact, complete means two SDK-zero replies, target_sequence/reason remain available. Transport loss returns Unknown, preserving earlier replies. |
| Snapshot.arms raw replies | Snapshot.arms are CallResult; Snapshot.faults retains harvested stop faults. Numeric validity still does not imply freshness. |
| close returns None | close returns CleanupReport; caller must check ok, stop.operations/faults and process/destructor facts. last_cleanup persists on the Session. |
| Controller.stop returns None | Returns StopReport; status.stop_result, status.last_pair and status.faults retain the known results. |
| Context may overwrite an earlier exception | Session context raises CleanupError(result) on cleanup failure only if there is no original exception; otherwise the original survives, with a note and last_cleanup. Destructors never throw. |

Four stop operation slots are left StopMotion, left ServoMoveEnd, right StopMotion, right ServoMoveEnd. Their states are NotDispatched, SdkReplied, Unknown or WorkerRejected. An absent reply has no synthetic vendor code. FaultRecord.arm is None for a session-wide transport failure, otherwise 0/1. The first per-operation worker failure stays latched even after a successful retry.

Normal close accepts per-operation progress frames before SDK destruction, then requires the destruction acknowledgement and normal exit/reap. forced is set only for a successful pidfd termination request. reaped/exit_code/term_signal/error preserve process evidence; sdk_destructors_completed=False means completion was not acknowledged, not that no destructor ran. Physical stopping is always unconfirmed by these software facts.

The Controller snapshots target, sequence, submission time and deadline under its state lock, then releases that lock before SDK waits. Per-arm deadline/stop checks immediately before enqueue are the dispatch linearization points. Concurrent stop after a check cannot revoke an already enqueued/entered SDK call. Mid-pair newer targets wait for the old transaction to finish within its original deadline; old/new values and sequences are not mixed.
