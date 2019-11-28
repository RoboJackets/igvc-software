# Navigation State Machine and Waypoint Orientation

*Issue #520 and #596*

**Author:**
- Matthew Hannay

## The Problem

The two problems this intends to solve are thus:
- We want to have more control over the interactions between `get_path`, `exe_path`, and `recovery`.
- We want the orientation of the robot at the waypoints to not cause the robot to take strange paths.

## Proposed Solution

The plan is to create a `navigation_server` node that will act as the middleman between `navigation_client`,
`move_base/get_path`, `move_base/exe_path`, and `move_base/recovery`. It will act as a state machine that will
act as such:

- Connect to all the `get_path`, `exe_path`, and `recovery` action clients
- Send the goal waypoint to `get_path` and wait for response
    - If SUCCESS:
        - If `preserve_waypoint_orientation`, send path to `exe_path`
        - Else, change the orientation of the waypoint pose to be in line with the previous pose, then send path to `exe_path`
    - If ABORTED:
        - If `use_recovery_behavior`, attempt pathing recovery behavior
        - Else, report what happened and abort goal
    - If PREEMPTED:
        - Report what happened, and cancel goal
    - If ANYTHING ELSE:
        - Report what happened and abort goal
- While `exe_path` runs, check on feedback
    - If oscillation is detected, cancel `exe_path` goal and run oscillation recovery behavior
- Interpret the result from `exe_path`
    - If SUCCESS:
        - Send the `navigation_client` a successful report
    - If ABORTED:
        - If `use_recovery_behavior`, attempt pathing recovery behavior
        - Else, report what happened and abort goal
    - If ANYTHING ELSE:
        - Report what happened 

## Questions & Research

How exactly should the state machine be implemented? The current implementation works by chaining functions together while
keeping track of the state without reason. Would a better implementation be a loop that continues until the goal is finished
that changes behavior depending on what state it is in, or a more simplified set of chained functions?

## Overall Scope

### Affected Packages

- `navigation_simulation` will be modified
- `navigation_client` will be modified
- `navigation_server` will be created

### Schedule

