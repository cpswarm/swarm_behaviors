#include "uav_tracking.h"

/**
 * @brief Callback of the action server which executes the tracking task until it is preempted or finished.
 * @param goal The goal message received from the action client.
 * @param as The action server offered by this node.
 */
void ActionCallback(const cpswarm_msgs::TrackingGoalConstPtr& goal, action_server_t* as)
{
    NodeHandle nh;

    // target id
    target.id = goal->target;

    // set loop rate
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);

    ROS_INFO("Executing tracking");

    // tracking library
    uav_simple_tracking uav_tracking(target, goal->altitude);
    uav_flocking_tracking* flocking;
    uav_simple_tracking* simple;
    if (behavior == "flocking")
        flocking = new uav_flocking_tracking(target, goal->altitude);
    else if (behavior == "simple")
        simple = new uav_simple_tracking(target, goal->altitude);
    else {
        ROS_ERROR("Unknown behavior %s, cannot perform tracking!", behavior.c_str());
        as->setAborted();
        return;
    }

    // execute tracking until state changes
    state = STATE_ACTIVE;
    while (ok() && !as->isPreemptRequested() && state == STATE_ACTIVE) {
        ROS_DEBUG("Tracking step");
        behavior_state_t result;
        if (behavior == "flocking")
            result = flocking->step(target);
        else if (behavior == "simple")
            result = simple->step(target);
        if (state == STATE_ACTIVE) {
            // stop tracking with certain probability
            double p;
            if (max_trackers == 0 || trackers <= min_trackers || max_trackers <  min_trackers)
                p = 0;
            else if (min_trackers == 0 || min_trackers == max_trackers) {
                p = 1;
            }
            else {
                p = 0.5 * log(trackers / min_trackers) / log(1.0 + (max_trackers - min_trackers) / (2 * min_trackers));
                p = min(p, 1.0);
            }

            if (rng->uniform01() < p)
                state = STATE_PREEMPTED;
            else
                state = result;
        }
        rate.sleep();
        spinOnce();
    }

    // stop moving
    if (behavior == "flocking") {
        flocking->stop();
        delete flocking;
    }
    else if (behavior == "simple") {
        simple->stop();
        delete simple;
    }

    // tracking succeeded
    if (state == STATE_SUCCEEDED) {
        ROS_INFO("Tracking succeeded");
        as->setSucceeded();
    }

    // tracking aborted
    else if (state == STATE_ABORTED) {
        ROS_INFO("Tracking aborted");
        as->setAborted();
    }

    // tracking was preempted
    else{
        ROS_INFO("Tracking preempted");
        as->setPreempted();
    }
}

/**
 * @brief Callback function to receive target update events.
 * @param msg ID and position of target.
 */
void update_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    // update information for this target
    if (msg->id == target.id)
        target = *msg;
}

/**
 * @brief Callback function to receive event that target has been lost.
 * @param msg ID and position of target.
 */
void lost_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    if (msg->id == target.id)
        state = STATE_ABORTED;
}

/**
 * @brief Callback function to receive event that target has been done.
 * @param msg ID and position of target.
 */
void done_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    if (msg->id == target.id)
        state = STATE_SUCCEEDED;
}

/**
 * @brief Callback function to receive number of UAVs tracking the same target.
 * @param msg ID number of trackers.
 */
void trackers_callback (const cpswarm_msgs::TargetTrackedBy::ConstPtr& msg)
{
    if (msg->id == target.id)
        trackers = msg->trackers;
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
    init(argc, argv, "uav_tracking");
    NodeHandle nh;

    // simultaneously tracking uavs
    trackers = 1;
    nh.param(this_node::getName() + "/max_trackers", max_trackers, 0);
    nh.param(this_node::getName() + "/min_trackers", min_trackers, 0);

    // tracking behavior
    nh.getParam(this_node::getName() + "/behavior", behavior);

    // initially, no targets being tracked
    target.id = -1;

    // subscribers
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    Subscriber update_sub = nh.subscribe("target_update", queue_size, update_callback);
    Subscriber lost_sub = nh.subscribe("target_lost", queue_size, lost_callback);
    Subscriber done_sub = nh.subscribe("target_done", queue_size, done_callback);

    // stop tracking probabilistically
    if (max_trackers != 0 && min_trackers <= max_trackers) {
        Subscriber trackers_sub = nh.subscribe("target_trackers", queue_size, trackers_callback);

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
    action_server_t as(nh, "uav_tracking", boost::bind(&ActionCallback, _1, &as), false);
    as.start();

    // wait for action client
    spin();

    return 0;
}

