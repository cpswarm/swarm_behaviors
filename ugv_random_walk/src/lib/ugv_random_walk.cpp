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
    occupied_sector_client = nh.serviceClient<cpswarm_msgs::get_occupied_sector>("obstacle_detection/get_occupied_sector");

    // initial direction as drone is placed
    direction.set(pos->get_yaw().rad());
    ROS_INFO("Initial direction %.2f", direction.rad_pos());

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
    direction.set(-atan2(b2.y - b1.y, b2.x - b1.x) - direction.rad());

    ROS_INFO("Changing direction %.2f", direction.rad_pos());
}

geometry_msgs::Pose ugv_random_walk::select_goal ()
{
    // compute goal position
    return pos->compute_goal(distance, direction.rad_pos());
}
