# uav_simple_tracking

This package tracks a target with an unmanned aerial vehicle (UAV). It is part of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* swarm_behaviors_position

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions/) are required:
* target_monitor

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [actionlib](https://wiki.ros.org/actionlib/)

## Execution
Run the launch file
```
roslaunch uav_simple_tracking uav_simple_tracking.launch
```
to launch the `uav_simple_tracking` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `uav_simple_tracking.yaml` that allows to configure the behavior of the `uav_simple_tracking` node.

## Nodes

### uav_simple_tracking
The `uav_simple_tracking` tracks a target with a UAV. The UAV moves to the position straight over the target. The position of the target is updated by the target monitor from the [swarm functions library](https://github.com/cpswarm/swarm_functions/). When the target is lost, i.e., the target is not in the camera field of view anymore, the tracking aborts. When the target is done, i.e., handled by another CPS, the tracking succeeds.

#### Action Goal
* `uav_tracking/goal` ([cpswarm_msgs/TrackingGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/Tracking.html))
  A goal that starts the tracking behavior. It contains the ID of the target to track.

#### Subscribed Topics
* `target_update` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Position updates of the target being tracked.
* `target_lost` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Whether the target being tracked has been lost.
* `target_done` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Whether the target being tracked has been handled by another CPS.

#### Parameters
* `~loop_rate` (real, default: `5.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.

## Code API
[uav_simple_tracking package code API documentation](https://cpswarm.github.io/swarm_behaviors/uav_simple_tracking/docs/html/files.html)
