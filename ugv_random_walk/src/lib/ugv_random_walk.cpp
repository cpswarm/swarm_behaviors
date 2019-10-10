#include "lib/ugv_random_walk.h"

ugv_random_walk::ugv_random_walk (int seed)
{
    NodeHandle nh;

    // init random number generator
    if (seed != 0) {
        rng = new random_numbers::RandomNumberGenerator(seed);
    }
    else {
        rng = new random_numbers::RandomNumberGenerator();
    }

    // init service clients
    bound_client = nh.serviceClient<cpswarm_msgs::ClosestBound>("area/closest_bound");
    bound_client.waitForExistence();
    clear_sector_client = nh.serviceClient<cpswarm_msgs::GetSector>("obstacle_detection/get_clear_sector");
    clear_sector_client.waitForExistence();

    // initial direction as drone is placed
    direction = pos.get_yaw();
    ROS_INFO("Initial direction %.2f", direction);

    // read parameters
    // get step size from optimized candidate
    nh.param(this_node::getName() + "/step_size", step_size, 3.0);
    ROS_INFO("Step size %.2f", step_size);
}

ugv_random_walk::~ugv_random_walk ()
{
    delete rng;
}

behavior_state_t ugv_random_walk::step ()
{
    // update position information
    spinOnce();

    // compute new goal
    geometry_msgs::Pose goal = select_goal();

    // reflect from bound
    if (pos.out_of_bounds(goal)) {
        if (reflect() == false)
            return STATE_ABORTED;
        goal = select_goal();
    }

    // move to new position
    pos.move(goal);

    // change direction
    if (new_direction() == false)
        return STATE_ABORTED;

    // return state to action server
    return STATE_ACTIVE;
}

bool ugv_random_walk::new_direction ()
{
    // get sector clear of obstacles and other uavs
    cpswarm_msgs::GetSector clear;
    if (clear_sector_client.call(clear) == false){
        ROS_ERROR("Failed to get clear sector");
        return false;
    }

    ROS_DEBUG("Clear [%.2f, %.2f] size %.2f", clear.response.min, clear.response.max, clear.response.max - clear.response.min);

    // generate random direction until one is found inside of area not occupied by obstacles
    geometry_msgs::Pose goal;
    do {
        direction = rng->uniformReal(clear.response.min, clear.response.max);
        goal = select_goal();
        ROS_DEBUG_THROTTLE(1, "Checking goal [%.2f, %.2f, %.2f] in direction %.2f...", goal.position.x, goal.position.y, goal.position.z, direction);
    } while (pos.out_of_bounds(goal));

    ROS_INFO("Changing direction %.2f", direction);

    return true;
}

bool ugv_random_walk::reflect ()
{
    // get boundary
    geometry_msgs::Point b1;
    geometry_msgs::Point b2;
    cpswarm_msgs::ClosestBound cb;
    cb.request.point = pos.get_pose().position;
    if (bound_client.call(cb)){
        b1 = cb.response.coords[0];
        b2 = cb.response.coords[1];
    }
    else{
        ROS_ERROR("Failed to get area boundary");
        return false;
    }

    // calculate reflection
    direction = remainder(-atan2(b2.y - b1.y, b2.x - b1.x) - direction, 2*M_PI);

    ROS_INFO("Changing direction %.2f", direction);

    return true;
}

geometry_msgs::Pose ugv_random_walk::select_goal ()
{
    // compute goal position
    return pos.compute_goal(step_size, direction);
}
