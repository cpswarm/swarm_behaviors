# uav_tracking
[![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__uav_tracking__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__uav_tracking__ubuntu_xenial__source/)

This package tracks a target with an unmanned aerial vehicle (UAV) using different types of algorithms. It is part of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* swarm_behaviors_flocking
* swarm_behaviors_position
* swarm_behaviors_velocity

The following packages of the [swarm functions library](https://github.com/cpswarm/swarm_functions/) are required:
* target_monitor

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)
* [actionlib](https://wiki.ros.org/actionlib/)
* [random_numbers](https://wiki.ros.org/random_numbers/)

## Execution
Run the launch file
```
roslaunch uav_tracking uav_tracking.launch
```
to launch the `uav_tracking` node.

The launch file can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `output` (string, default: `screen`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).
* `behavior` (string)
  The behavior to use for tracking. See below for more details.

In the `param` subdirectory there is the parameter file `uav_tracking.yaml` that allows to configure the behavior of the `uav_tracking` node.

## Nodes

### uav_tracking
The `uav_tracking` lets a swarm of UAVs track a target. The position of the target is updated by the target monitor from the [swarm functions library](https://github.com/cpswarm/swarm_functions/). It provides an action server that has three outcomes: `succeeded`, `preempted`, or `aborted`. When the target is lost, i.e., the target is not in the camera field of view anymore, the tracking aborts. When the target is done, i.e., handled by another CPS, the tracking succeeds.
If the parameters `max_trackers` and `min_trackers` are set according to

    0 <= min_trackers <= max_trackers
    0 < max_trackers

the tracking is preempted with a certain probability

    p = min(1, 0.5 ⋅ log(trackers / min_trackers) / log(1.0 + (max_trackers - min_trackers) / (2 ⋅ min_trackers)))

where `trackers` is the number of UAVs currently tracking the same target. The tracking is performed either individually or cooperatively by employing one of the following algorithms:
* **Flocking**: The UAVs create a formation while tracking the target.
* **Simple**: A single UAV moves to the position straight over the target.

#### Action Goal
* `uav_tracking/goal` ([cpswarm_msgs/TrackingGoal](https://cpswarm.github.io/cpswarm_msgs/html/action/Tracking.html))
  A goal that starts the tracking behavior. It contains the ID of the target to track and the altitude at which to operate.

#### Subscribed Topics
* `target_update` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Position updates of the target being tracked.
* `target_lost` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Whether the target being tracked has been lost.
* `target_done` ([cpswarm_msgs/TargetPositionEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetPositionEvent.html))
  Whether the target being tracked has been handled by another CPS.
* `target_trackers` ([cpswarm_msgs/TargetTrackedBy](https://cpswarm.github.io/cpswarm_msgs/html/msg/TargetTrackedBy.html))
  The number of UAVs tracking the same target.

#### Parameters
* `~loop_rate` (real, default: `5.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~max_trackers` (integer, default: `0`)
  The maximum number of UAVs that should be tracking the same target simultaneously. 0 disables check.
* `~min_trackers` (integer, default: `0`)
  The number of UAVs tracking the same target above which a UAV stops tracking with a certain probability.
* `/rng_seed` (integer, default: `0`)
  The seed used for random number generation. In the default case, a random seed is generated.

## Code API
[uav_tracking package code API documentation](https://cpswarm.github.io/swarm_behaviors/uav_tracking/docs/html/files.html)
