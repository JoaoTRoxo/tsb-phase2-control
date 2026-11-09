# tsb_control_msgs

Messages used by `tsb_control`. They are defined only where no standard message carries the
meaning: wheel commands, odometry, IMU data and paths all use standard types.

| Message | Topic | Purpose |
| --- | --- | --- |
| `VesselState` | `/state` | Fused state: position, heading, surge, yaw rate, validity |
| `Reference` | `/desired` | The `[desired surge, desired heading]` vector, with named fields |
| `PidTerms` | inside `PidDebug` | Every internal term of one PID loop |
| `PidDebug` | `/pid/debug` | `PidTerms` for both loops, for plotting and debugging |
| `AllocationFeedback` | `/allocation/feedback` | How much of each command the wheels delivered |
| `PathStatus` | `/path_following/status` | Active segment, cross-track error, finished flag |

Field-by-field documentation is in the comments of each file in `msg/`.
