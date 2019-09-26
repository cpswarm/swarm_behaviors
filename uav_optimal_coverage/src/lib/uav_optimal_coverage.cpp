#include "lib/uav_optimal_coverage.h"

uav_optimal_coverage::uav_optimal_coverage () : uav_coverage()
{
    nh.param(this_node::getName() + "/target_velocity", target_velocity, 0.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/tolerance", tolerance, 0.5);

    // publishers, subscribers, and service clients
    ServiceClient wp_getter = nh.serviceClient<cpswarm_msgs::GetWaypoint>("coverage_path/waypoint");
    wp_getter.waitForExistence();

    // get initial waypoint of path
    cpswarm_msgs::GetWaypoint wp;
    wp.request.tolerance = tolerance;
    if (wp_getter.call(wp) == false){
        ROS_ERROR("Failed to get waypoint, cannot perform coverage!");
        return;
    }
    waypoint = wp.response.point;
    ROS_DEBUG("Initial waypoint [%.2f, %.2f]", waypoint.x, waypoint.y);
}

uav_optimal_coverage::~uav_optimal_coverage ()
{
}

behavior_state_t uav_optimal_coverage::step ()
{
    // update position information
    spinOnce();

    // update information about tracking targets
    sar_targets->update();

    if (state == STATE_ACTIVE) {
        // reached current waypoint of path
        if (hypot(waypoint.x - pos->get_pose().position.x, waypoint.y - pos->get_pose().position.y) < tolerance) {
            // get next waypoint of path
            cpswarm_msgs::GetWaypoint wp;
            wp.request.tolerance = tolerance;
            if (wp_getter.call(wp) == false){
                ROS_ERROR("Failed to get waypoint, cannot perform coverage!");
                return STATE_ABORTED;
            }
            waypoint = wp.response.point;
            ROS_DEBUG("Move to waypoint [%.2f, %.2f]", waypoint.x, waypoint.y);
        }

        // finished path
        if (waypoint.x == 0.0 && waypoint.y == 0.0)
            return STATE_SUCCEEDED;

        // compute velocity to reach waypoint
        geometry_msgs::Vector3 velocity = vel->compute_velocity(waypoint, target_velocity);

        // move with new velocity
        move(velocity);
    }

    // return state to action server
    return state;
}
