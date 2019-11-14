# position

This package provides position related functionalities. It is a library package of the swarm behaviors library.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* area_provider
* *_pos_provider
* *_pos_controller

Further required packages are:
* [tf2](https://wiki.ros.org/tf2/)

## Libraries

### position
The `position` library provides position related functionalities. These functionalities can be grouped into three categories: calculations, retrieval of sensor data, and setting of actuators. The calculations include distance measurements, checking whether a position is in the defined environment, and calculation of goal positions. The sensor data includes absolute and relative bearing as well as local position. The actuators to be controlled is locomotion by providing a goal position waypoint.

#### Subscribed Topics
* `pos_provider/pose` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  Position and orientation of the CPS.

#### Published Topics
* `pos_controller/goal_position` ([geometry_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  The goal waypoint where the CPS shall move to.

#### Services Called
* `area/out_of_bounds` ([cpswarm_msgs/OutOfBounds](https://cpswarm.github.io/cpswarm_msgs/html/srv/OutOfBounds.html))
  Check whether a coordinate is within the defined environment area.

#### Parameters
* `~loop_rate` (real, default: 5.0)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: 1)
  The size of the message queue used for publishing and subscribing to topics.
* `~goal_timeout` (real, default: 30.0)
  The time in seconds that the CPS is given time to reach a destination before giving up.
* `~goal_tolerance` (real, default: 0.1)
  The distance in meter that the CPS can be away from a goal while still being considered to have reached that goal.
* `~yaw_tolerance` (real, default: 0.02)
  The angle in radian that the CPS can be away from a goal while still being considered to have reached that goal.
* `~turning` (boolean, default: true)
  Whether the CPS should turn its front into movement direction or not.

## Code API
[position package code API documentation](https://cpswarm.github.io/swarm_behaviors/position/docs/html/files.html)
