#include "lib/uav_optimal_coverage.h"

uav_optimal_coverage::uav_optimal_coverage (double altitude) : pos(altitude)
{
    NodeHandle nh;

    // init state of algorithm
    state = STATE_ACTIVE;

    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // publishers, subscribers, and service clients
    wp_getter = nh.serviceClient<cpswarm_msgs::GetWaypoint>("coverage_path/waypoint");
    wp_getter.waitForExistence();

    // get initial waypoint of path
    get_wp.request.position = pos.get_pose().position;
    get_wp.request.tolerance = pos.get_tolerance();
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
        if (pos.reached()) {
            // get next waypoint of path
            get_wp.request.position = pos.get_pose().position;
            get_wp.request.tolerance = pos.get_tolerance();
            if (wp_getter.call(get_wp) == false) {
                ROS_ERROR("Failed to get waypoint, cannot perform coverage!");
                return STATE_ABORTED;
            }

            // finished path
            if (get_wp.response.valid == false) {
                ROS_INFO("Path completely traversed, stop coverage!");
                return STATE_ABORTED;
            }

            waypoint = get_wp.response.point;
            ROS_INFO("Move to waypoint [%.2f, %.2f]", waypoint.x, waypoint.y);
        }

        // convert waypoint to pose
        geometry_msgs::Pose goal = pos.get_pose();
        goal.position.x = waypoint.x;
        goal.position.y = waypoint.y;

        // move to new position
        if (pos.move(goal) == false)
            state = STATE_ABORTED;
    }

    // return state to action server
    return state;
}

void uav_optimal_coverage::stop ()
{
    pos.stop();
}
