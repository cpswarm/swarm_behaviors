# swarm_behaviors_velocity

This package provides velocity related functionalities. It is a library package of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following library packages of the [swarm behaviors library](https://github.com/cpswarm/swarm_behaviors) are required:
* swarm_behaviors_position

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* *_vel_provider
* *_vel_controller

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)

## Libraries

### velocity
The `velocity` library provides velocity related functionalities. These include retrieval of current velocity, computation of relative velocity, computation of velocity to reach a certain waypoint, and setting the actuators velocity.

#### Subscribed Topics
* `vel_provider/velocity` ([geometry_msgs/TwistStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  Current velocity of the CPS.

#### Published Topics
* `vel_controller/target_velocity` ([geometry_msgs/Twist](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The target velocity at which the CPS shall move.

#### Parameters
* `~loop_rate` (real, default: `5.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.

## Code API
[swarm_behaviors_velocity package code API documentation](https://cpswarm.github.io/swarm_behaviors/lib/swarm_behaviors_velocity/docs/html/files.html)
