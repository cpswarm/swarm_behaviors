#include "lib/uav_random_direction.h"

uav_random_direction::uav_random_direction (int seed) : uav_coverage()
{
    // init random number generator
    if (seed != 0) {
        rng = new random_numbers::RandomNumberGenerator(seed);
    }
    else {
        rng = new random_numbers::RandomNumberGenerator();
    }

    // init service clients
    occupied_sector_client = nh.serviceClient<cpswarm_msgs::get_occupied_sector>("obstacle_detection/get_occupied_sector");

    // inititial direction as drone is placed
    direction.set(pos->get_yaw().rad());

    ROS_INFO("Initial direction %.2f", direction.rad_pos());
}

uav_random_direction::~uav_random_direction ()
{
    delete rng;
}

behavior_state_t uav_random_direction::step ()
{
    // update position information
    spinOnce();

    // update information about tracking targets
    sar_targets->update();

    if (state == STATE_ACTIVE) {

        // compute new goal with maximum distance
        distance = step_size_max;
        geometry_msgs::Pose goal = select_goal();

        // new goal is out of bounds
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

            // change direction
            if (pos->out_of_bounds(goal)) {
                new_direction();
                goal = select_goal();
            }
        }

        // move to new position
        move(goal);
    }

    // return state to action server
    return state;
}

void uav_random_direction::obstacle_avoidance ()
{
    ROS_ERROR("Obstacle avoidance");

    // change direction
    new_direction();
}

geometry_msgs::Pose uav_random_direction::select_goal ()
{
    // compute goal position
    return pos->compute_goal(distance, direction.rad_pos());
}

void uav_random_direction::new_direction ()
{
    // get sector clear of obstacles and other uavs
    sector* occ;
    cpswarm_msgs::get_occupied_sector gos;
    if (occupied_sector_client.call(gos)){
        occ = new sector(gos.response.min, gos.response.max);
    }
    else{
        ROS_ERROR("Failed to get occupied sector");
        return;
    }
    sector clear =occ->inverse();
    delete occ;

    ROS_DEBUG("Clear [%.2f, %.2f] size %.2f", clear.min_ord(), clear.max_ord(), clear.size());

    // generate random direction until one is found inside of area not occupied by obstacles
    geometry_msgs::Pose goal;
    do {
        direction.set(rng->uniformReal(clear.min_ord(), clear.max_ord()));
        goal = select_goal();
        ROS_DEBUG_THROTTLE(1, "Checking goal [%.2f, %.2f, %.2f] in direction %.2f...", goal.position.x, goal.position.y, goal.position.z, direction.rad_pos());
    } while (pos->out_of_bounds(goal));

    ROS_INFO("Changing direction %.2f", direction.rad_pos());
}
