#include "lib/uav_optimal_coverage.h"

uav_optimal_coverage::uav_optimal_coverage ()
{
    NodeHandle nh;

    // init state of algorithm
    state = STATE_ACTIVE;

    // read parameters
    nh.param(this_node::getName() + "/target_velocity", target_velocity, 0.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/tolerance", tolerance, 0.5);

    // publishers, subscribers, and service clients
    wp_getter = nh.serviceClient<cpswarm_msgs::GetWaypoint>("coverage_path/waypoint");
    wp_getter.waitForExistence();

    // get initial waypoint of path
    get_wp.request.tolerance = tolerance;
    if (wp_getter.call(get_wp) == false || get_wp.response.valid == false){
        ROS_ERROR("Failed to get waypoint, cannot perform coverage!");
        state = STATE_ABORTED;
        return;
    }
    waypoint = get_wp.response.point;
    ROS_INFO("Initial waypoint [%.2f, %.2f]", waypoint.x, waypoint.y);
}

uav_optimal_coverage::~uav_optimal_coverage ()
{
}

behavior_state_t uav_optimal_coverage::step ()
{
    // update position information
    spinOnce();

    if (state == STATE_ACTIVE) {
        // reached current waypoint of path
        if (hypot(waypoint.x - pos.get_pose().position.x, waypoint.y - pos.get_pose().position.y) < tolerance) {
            // get next waypoint of path
            get_wp.request.tolerance = tolerance;
            if (wp_getter.call(get_wp) == false){
                ROS_ERROR("Failed to get waypoint, cannot perform coverage!");
                return STATE_ABORTED;
            }
            waypoint = get_wp.response.point;
            ROS_INFO("Move to waypoint [%.2f, %.2f]", waypoint.x, waypoint.y);
        }

        // compute velocity to reach waypoint
        geometry_msgs::Vector3 velocity;
        if (get_wp.response.valid)
            velocity = vel.compute_velocity(waypoint, target_velocity);

        // finished path
        else
            state = STATE_SUCCEEDED;


        // move with new velocity
        vel.move(velocity);
    }

    // return state to action server
    return state;
}
