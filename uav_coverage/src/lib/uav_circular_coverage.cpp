#include "lib/uav_circular_coverage.h"

uav_circular_coverage::uav_circular_coverage (double altitude) : uav_coverage_behavior(altitude)
{
    // read parameters
    NodeHandle nh;
    nh.param(this_node::getName() + "/circular/fov_hor", fov_hor, 1.236);
    nh.param(this_node::getName() + "/circular/fov_ver", fov_ver, 0.970);
    nh.param(this_node::getName() + "/circular/steps", max_steps, 20);

    // init number of search steps
    steps = 0;

    // init circular coordinate origin
    double distance, direction;
    compute_circle(distance, direction);

    // invert direction to reach origin from current pose
    direction += M_PI;
    origin = pos.get_pose();
}

behavior_state_t uav_circular_coverage::step ()
{
    // update position information
    spinOnce();

    // next search step
    if (pos.reached()) {
        ros::Duration(1).sleep(); // Here is where we can insert the command to take a picture for instance
        if (steps == 0)
            ROS_INFO("Ready to start circle");
        ++steps;

        ROS_INFO("Number of steps is %d", steps);

        // reached maximum number of steps, stop circular coverage
        if (steps >= max_steps+3) {                         //=========Samira: I have more than max step sizes here so we have the circle complete.
            ROS_INFO("Reached maximum coverage steps. Circle complete!");
            return STATE_SUCCEEDED;
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

void uav_circular_coverage::compute_circle (double &distance, double &direction)
{
    // radius of circle
    double r = altitude * tan(fov_hor / 2) / M_PI;

    double x = r * cos(2*M_PI * (steps-1) / max_steps);
    double y = r * sin(2*M_PI * (steps-1) / max_steps);

    // compute distance and heading
    distance = hypot(x, y);
    direction = atan2(y, x); // towards circle center, not direction of motion
}

geometry_msgs::Pose uav_circular_coverage::select_goal ()
{
    // compute heading and distance for current step
    double distance, direction;
    compute_circle(distance, direction);

    // compute gps coordinats of goal position
    return pos.compute_goal(origin, distance, direction);

}
