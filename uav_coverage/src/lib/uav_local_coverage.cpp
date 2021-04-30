#include "lib/uav_local_coverage.h"

uav_local_coverage::uav_local_coverage (double altitude) : uav_coverage_behavior(altitude)
{
    // read parameters
    NodeHandle nh;
    nh.param(this_node::getName() + "/local/fov_hor", fov_hor, 1.236);
    nh.param(this_node::getName() + "/local/fov_ver", fov_ver, 0.970);
    nh.param(this_node::getName() + "/local/steps", max_steps, 20);

    // init number of search steps
    steps = 0;

    // init local coordinate origin
    // it is the center of the circle that defines the involute
    double distance, direction;
    compute_involute(distance, direction);
    // invert direction to reach origin from current pose
    // the current pose is the 0th step on the involute (x = radius a, y = 0)
    direction += M_PI;
    origin = pos.get_pose();
}

behavior_state_t uav_local_coverage::step ()
{
    // update position information
    spinOnce();

    // next search step
    if (pos.reached()) {
        ++steps;

        // reached maximum number of steps, stop local coverage
        if (steps >= max_steps) {
            ROS_INFO("Reached maximum coverage steps!");
            return STATE_ABORTED;
        }
    }

    // compute goal
    geometry_msgs::Pose goal = select_goal();

    // move to new goal
    if (pos.move(goal) == false)
        return STATE_ABORTED;

    // return state to action server
    return STATE_ACTIVE;
}

void uav_local_coverage::compute_involute (double &distance, double &direction)
{
    // parameters for circle involute
    double a = altitude * tan(fov_hor / 2) / M_PI; // radius
    double b = altitude * tan(fov_ver / 2) * 2;    // step size

    // compute local coordinates using involute
    double s = b * steps; // arc length
    double t = sqrt(2 * s / a); // tangential angle
    double x = a * (cos(t) + t * sin(t));
    double y = a * (sin(t) - t * cos(t));

    // compute distance and heading from local origin
    distance = hypot(x, y);
    direction = atan2(y, x);
}

geometry_msgs::Pose uav_local_coverage::select_goal ()
{
    // compute heading and distance for current step
    double distance, direction;
    compute_involute(distance, direction);

    // compute gps coordinats of goal position
    return pos.compute_goal(origin, distance, direction);
}
