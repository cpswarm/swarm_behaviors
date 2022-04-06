#include "lib/uav_random_coverage.h"

uav_random_coverage::uav_random_coverage (double altitude) : uav_coverage_behavior(altitude)
{
    NodeHandle nh;

    // read parameters
    nh.param(this_node::getName() + "/random/margin", margin, 0.5);
    nh.param(this_node::getName() + "/random/max_tries", max_tries, 10);

    // init random number generator
    int seed;
    nh.param<int>("/rng_seed", seed, 0);
    if (seed != 0) {
        rng = new random_numbers::RandomNumberGenerator(seed);
    }
    else {
        rng = new random_numbers::RandomNumberGenerator();
    }

    // init service clients
    area_client = nh.serviceClient<cpswarm_msgs::GetPoints>("area/get_area");
    area_client.waitForExistence();
    clear_sector_client = nh.serviceClient<cpswarm_msgs::GetSector>("obstacle_detection/get_clear_sector");
    clear_sector_client.waitForExistence();

    // inititial direction as drone is placed
    direction = pos.get_yaw();

    ROS_INFO("Initial direction %.2f", direction);

    // initialize goal
    select_goal();
}

uav_random_coverage::~uav_random_coverage ()
{
    delete rng;
}

behavior_state_t uav_random_coverage::step ()
{
    // update position information
    spinOnce();

    // move to goal position
    pos.move(goal);

    // at boundary
    if (pos.reached()) {
        if (new_direction() == false)
            return STATE_ABORTED;
    }

    // obstacle in direction of new goal
    if (pos.occupied(goal)) {
        ROS_DEBUG("Obstacle ahead!");
        // change direction
        if (new_direction() == false)
            return STATE_ABORTED;
    }

    return STATE_ACTIVE;
}

bool uav_random_coverage::select_goal ()
{
    // calculate goal
    cpswarm_msgs::GetPoints area;
    if (area_client.call(area)){
        // get area polygon
        vector<geometry_msgs::Point> coords = area.response.points;

        // find intersecting point of direction and area boundary
        for (int i = 0; i < coords.size(); ++i) {
            geometry_msgs::Point v1;
            v1.x = goal.position.x - coords[i].x;
            v1.y = goal.position.y - coords[i].y;

            geometry_msgs::Point v2;
            v2.x = coords[(i+1)%coords.size()].x - coords[i].x;
            v2.y = coords[(i+1)%coords.size()].y - coords[i].y;

            geometry_msgs::Point v3;
            v3.x = -sin(direction);
            v3.y = cos(direction);

            ROS_DEBUG("Calculate intersection with boundary (%.2f,%.2f)--(%.2f,%.2f)", coords[i].x, coords[i].y, coords[(i+1)%coords.size()].x, coords[(i+1)%coords.size()].y);

            double dot1 = v1.x*v3.x + v1.y*v3.y;
            double dot2 = v2.x*v3.x + v2.y*v3.y;
            double cross = v2.x*v1.y - v1.x*v2.y;

            double t1 = cross / dot2;
            double t2 = dot1 / dot2;

            if (t1 >= 0.0 && t2 >= 0.0 && t2 <= 1.0) {
                goal.position.x += (t1 - margin) * cos(direction);
                goal.position.y += (t1 - margin) * sin(direction);
                ROS_DEBUG("Selected goal (%.2f,%2.f)", goal.position.x, goal.position.y);
                return true;
            }
        }

        ROS_ERROR("Failed to compute goal");
    }
    else{
        ROS_ERROR("Failed to get area");
    }

    return false;
}

bool uav_random_coverage::new_direction ()
{
    // get sector clear of obstacles and other cpss
    cpswarm_msgs::GetSector clear;
    if (clear_sector_client.call(clear) == false){
        ROS_ERROR("Failed to get clear sector");
        return false;
    }

    ROS_DEBUG("Clear [%.2f, %.2f] size %.2f", clear.response.min, clear.response.max, clear.response.max - clear.response.min);

    // generate random direction until one is found inside of area not occupied by obstacles
    for (int i=0; i<max_tries; ++i) {
        // change direction
        direction = rng->uniformReal(clear.response.min, clear.response.max);
        ROS_DEBUG("Checking direction %.2f...", direction);

        // try selecting goal in that direction
        if (select_goal() && pos.out_of_bounds(goal) == false)
            break;
    }

    ROS_INFO("Changing direction %.2f", direction);

    return true;
}
