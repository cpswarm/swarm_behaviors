#include "lib/uav_flocking_coverage.h"

uav_flocking_coverage::uav_flocking_coverage ()
{
    NodeHandle nh;
    flock = new uav_flocking();
    nh.param(this_node::getName() + "/flock_vel", flock_vel, 0.5);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    geometry_msgs::Point wp;// = path->current_wp(); TODO
    ROS_DEBUG("Initial waypoint: [%.2f, %.2f]", wp.x, wp.y);

    // publishers for visualization
    pub_vis_wp = nh.advertise<geometry_msgs::PointStamped>("vis/wp", queue_size);
}

uav_flocking_coverage::~uav_flocking_coverage ()
{
    delete flock;
}

behavior_state_t uav_flocking_coverage::step ()
{
    // update position information
    spinOnce();

    // get current waypoints of path
    geometry_msgs::Point wp;// = path->current_wp(); TODO

    // compute distance reached within next few cycles
    int cycles = 10; // TODO: make param
    double delta = 10;//cycles * flock_vel * rate->expectedCycleTime().toSec();

    // select next waypoint if close to current one
    // TODO: should be enough if one of the swarm members reaches the point?! or swarm center?
    geometry_msgs::Pose wp_pose;
    wp_pose.position = wp;
    if (pos.dist(wp_pose) < delta) {
//             wp = path->next_wp();
        ROS_DEBUG("Move to waypoint [%.2f, %.2f]", wp.x, wp.y);
    }

    // visualize waypoint
    geometry_msgs::PointStamped wp_st;
    wp_st.point = wp;
    wp_st.header.stamp = Time::now();
    wp_st.header.frame_id = "local_origin_ned";
    pub_vis_wp.publish(wp_st);

    // finished path
    if (wp.x == 0.0 && wp.y == 0.0)
        return STATE_SUCCEEDED;

    // compute velocity to reach waypoint
    geometry_msgs::Vector3 cover_velocity = vel.compute_velocity(wp, flock_vel);

    // compute velocity for flocking of CPSs
    geometry_msgs::Vector3 velocity = flock->coverage(cover_velocity);

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
