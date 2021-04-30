#include "uav_coverage.h"

/**
 * @brief Callback of the action server which executes the coverage task until it is preempted or finished.
 * @param goal The goal message received from the action client.
 * @param as The action server offered by this node.
 */
void ActionCallback (const cpswarm_msgs::CoverageGoalConstPtr& goal, action_server_t* as)
{
    NodeHandle nh;

    // set loop rate
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);

    ROS_INFO("Executing coverage");

    // coverage library
    uav_flocking_coverage* flocking;
    uav_local_coverage* local;
    uav_random_coverage* random;
    uav_systematic_coverage* systematic;
    if (behavior == "flocking")
        flocking = new uav_flocking_coverage(goal->altitude);
    else if (behavior == "local")
        local = new uav_local_coverage(goal->altitude);
    else if (behavior == "random")
        random = new uav_random_coverage(goal->altitude);
    else if (behavior == "systematic")
        systematic = new uav_systematic_coverage(goal->altitude);
    else {
        ROS_ERROR("Unknown behavior %s, cannot perform coverage!", behavior.c_str());
        as->setAborted();
        return;
    }

    // execute coverage until state changes
    state = STATE_ACTIVE;
    while (ok() && !as->isPreemptRequested() && state == STATE_ACTIVE) {
        behavior_state_t result;
        if (behavior == "flocking")
            result = flocking->step();
        else if (behavior == "local")
            result = local->step();
        else if (behavior == "random")
            result = random->step();
        else if (behavior == "systematic")
            result = systematic->step();
        if (state == STATE_ACTIVE)
            state = result;
        rate.sleep();
        spinOnce();
    }

    // stop moving
    if (behavior == "flocking") {
        flocking->stop();
        delete flocking;
    }
    else if (behavior == "local") {
        local->stop();
        delete local;
    }
    else if (behavior == "random") {
        random->stop();
        delete random;
    }
    else if (behavior == "systematic") {
        systematic->stop();
        delete systematic;
    }

    // coverage succeeded
    if (state == STATE_SUCCEEDED) {
        ROS_INFO("Coverage succeeded, found target %d at [%f, %f]", result.target_id, result.target_pose.pose.position.x, result.target_pose.pose.position.y);

        as->setSucceeded(result);
    }

    // coverage aborted
    else if (state == STATE_ABORTED) {
        ROS_INFO("Coverage aborted");
        as->setAborted();
    }

    // coverage was preempted
    else{
        ROS_INFO("Coverage preempted, help with target %d at [%f, %f]", result.target_id, result.target_pose.pose.position.x, result.target_pose.pose.position.y);
        as->setPreempted();
    }
}

/**
 * @brief Callback function to receive details of a target that has been detected.
 * @param msg ID and position of target.
 */
void found_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    state = STATE_SUCCEEDED;
    result.target_id = msg->id;
    result.target_pose = msg->pose;
}

/**
 * @brief Callback function to receive position of this CPS.
 * @param msg Local position.
 */
void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = msg->pose;
}

/**
 * @brief Callback function to receive help calls for targets found by other CPSs.
 * @param msg ID, position, and times of the target.
 */
void help_callback (const cpswarm_msgs::TargetHelp::ConstPtr& msg)
{
    // distance to target
    double dist = hypot(msg->pose.pose.position.x - pose.position.x, msg->pose.pose.position.y - pose.position.y);

    // relative time the target still needs help
    double time;
    if (msg->time_need > 0)
        time = msg->time_avail / msg->time_need;
    else
        time = 1;

    // probability for helping
    double p = exp(dist * 2 * log(0.5) / help_range * time);
    double rand = rng->uniform01();
    if (rand < p) {
        state = STATE_PREEMPTED;
        result.target_id = msg->id;
        result.target_pose = msg->pose;
    }
}

/**
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char** argv)
{
    // init ros node
    init(argc, argv, "uav_coverage");
    NodeHandle nh;

    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.getParam(this_node::getName() + "/behavior", behavior);

    // stop covering once a target has been found
    bool single_target;
    nh.param(this_node::getName() + "/single_target", single_target, true);
    Subscriber found_sub;
    if (single_target)
        found_sub = nh.subscribe("target_found", queue_size, found_callback);

    // help other cpss
    nh.param(this_node::getName() + "/help_range", help_range, 0.0);
    Subscriber help_sub, pose_sub;
    if (help_range > 0) {
        // subscribe to relevant topics
        help_sub = nh.subscribe("target_help", queue_size, help_callback);
        pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);

        // init random number generator
        int seed;
        nh.param<int>("/rng_seed", seed, 0);
        if (seed != 0) {
            rng = new random_numbers::RandomNumberGenerator(seed);
        }
        else {
            rng = new random_numbers::RandomNumberGenerator();
        }
    }

    // start action server
    action_server_t as(nh, "uav_coverage", boost::bind(&ActionCallback, _1, &as), false);
    as.start();

    // wait for action client
    spin();

    // clean up
    delete rng;

    return 0;
}