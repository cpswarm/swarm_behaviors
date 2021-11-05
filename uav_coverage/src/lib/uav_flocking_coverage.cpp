#include "lib/uav_flocking_coverage.h"

uav_flocking_coverage::uav_flocking_coverage (double altitude) : uav_coverage_behavior(altitude), flock(altitude), vel(altitude)
{
    NodeHandle nh;
    nh.param(this_node::getName() + "/flocking/flock_vel", flock_vel, 0.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // publishers, subscribers, and service clients
    wp_getter = nh.serviceClient<cpswarm_msgs::GetWaypoint>("coverage_path/waypoint");
    wp_getter.waitForExistence();

    // initial waypoint
    waypoint = flock.center();
    ROS_DEBUG("Initial waypoint: [%.2f, %.2f]", waypoint.x, waypoint.y);
}

behavior_state_t uav_flocking_coverage::step ()
{
    // update position information
    spinOnce();

    // reached current waypoint of path
    geometry_msgs::PoseStamped wpp;
    wpp.header.stamp = ros::Time::now();
    wpp.pose.position = waypoint;
    if (pos.reached(wpp)) {
        // get next waypoint of path
        get_wp.request.position = flock.center();
        get_wp.request.tolerance = pos.get_tolerance();
        if (wp_getter.call(get_wp) == false) {
            ROS_ERROR("Failed to get waypoint, cannot perform coverage!");
            return STATE_ABORTED;
        }

        // finished path
        if (get_wp.response.valid == false) {
            ROS_INFO("Path completely traversed, stop coverage!");
            return STATE_SUCCEEDED;
        }

        waypoint = get_wp.response.point;
        ROS_DEBUG("Move to waypoint [%.2f, %.2f]", waypoint.x, waypoint.y);
    }

    // compute velocity to reach waypoint
    geometry_msgs::Vector3 cover_velocity = vel.compute_velocity(waypoint, flock_vel);

    // compute velocity for flocking of CPSs
    geometry_msgs::Vector3 velocity = flock.flocking_velocity(cover_velocity);

    // move with new velocity
    vel.move(velocity);

    // return state to action server
    return STATE_ACTIVE;
}

void uav_flocking_coverage::stop ()
{
    geometry_msgs::Vector3 velocity;
    vel.move(velocity);
}
