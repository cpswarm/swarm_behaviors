#include "lib/ugv_random_walk.h"

ugv_random_walk::ugv_random_walk (int seed) : ugv_coverage()
{
    // init random number generator
    if (seed != 0) {
        rng = new random_numbers::RandomNumberGenerator(seed);
    }
    else {
        rng = new random_numbers::RandomNumberGenerator();
    }

    // init service clients
    bound_client = nh.serviceClient<cpswarm_msgs::closest_bound>("area/closest_bound");
    clear_sector_client = nh.serviceClient<cpswarm_msgs::get_sector>("obstacle_detection/get_clear_sector");

    // initial direction as drone is placed
    direction = pos->get_yaw();
    ROS_INFO("Initial direction %.2f", direction);

    // get step size from optimized candidate
//     step_size_max = TODO
    ROS_INFO("Step size %.2f", step_size_max);
}

ugv_random_walk::~ugv_random_walk ()
{
    delete rng;
}

behavior_state_t ugv_random_walk::step ()
{
    // update position information
    spinOnce();

    // update information about tracking targets
    sar_targets->update();

    if (state == STATE_ACTIVE) {

        // compute new goal with maximum distance
        distance = step_size_max;
        geometry_msgs::Pose goal = select_goal();

        // try to go closer to area bound
        if (pos->out_of_bounds(goal)) {
            // delta distance
            double step = (step_size_max - step_size_min) / 10;

            // look for largest possible distance
            for (int i = 0; i <= 10; ++i) {
                // reduce distance
                distance = step_size_max - i * step;

                // select goal at new distance
                goal = select_goal();

                // found valid goal
                if (!pos->out_of_bounds(goal))
                    break;
            }

            // reflect from bound
            if (pos->out_of_bounds(goal)) {
                reflect();
                goal = select_goal();
            }
        }

        // move to new position
        move(goal);

        // change direction
        new_direction();
    }

    // return state to action server
    return state;
}

void ugv_random_walk::new_direction ()
{
    // get sector clear of obstacles and other uavs
    cpswarm_msgs::get_sector clear;
    if (clear_sector_client.call(clear)){
        ROS_ERROR("Failed to get clear sector");
        return;
    }

    ROS_DEBUG("Clear [%.2f, %.2f] size %.2f", clear.response.min, clear.response.max, clear.response.max - clear.response.min);

    // generate random direction until one is found inside of area not occupied by obstacles
    geometry_msgs::Pose goal;
    do {
        direction = rng->uniformReal(clear.response.min, clear.response.max);
        goal = select_goal();
        ROS_DEBUG_THROTTLE(1, "Checking goal [%.2f, %.2f, %.2f] in direction %.2f...", goal.position.x, goal.position.y, goal.position.z, direction);
    } while (pos->out_of_bounds(goal));

    ROS_INFO("Changing direction %.2f", direction);
}

void ugv_random_walk::reflect ()
{
    // get boundary
    geometry_msgs::Point b1;
    geometry_msgs::Point b2;
    cpswarm_msgs::closest_bound cb;
    cb.request.point = pos->get_pose().position;
    if (bound_client.call(cb)){
        b1 = cb.response.coords[0];
        b2 = cb.response.coords[1];
    }
    else{
        ROS_ERROR("Failed to get area boundary");
        return;
    }

    // calculate reflection
    direction = remainder(-atan2(b2.y - b1.y, b2.x - b1.x) - direction, 2*M_PI);

    ROS_INFO("Changing direction %.2f", direction);
}

geometry_msgs::Pose ugv_random_walk::select_goal ()
{
    // compute goal position
    return pos->compute_goal(distance, direction);
}
