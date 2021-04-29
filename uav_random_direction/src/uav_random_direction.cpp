#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <cpswarm_msgs/CoverageAction.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include <cpswarm_msgs/TargetHelp.h>
#include "lib/uav_random_direction.h"

using namespace ros;

/**
 * @brief The state of the behavior algorithm.
 */
behavior_state_t state;

/**
 * @brief The target found during execution of the coverage algorithm.
 */
cpswarm_msgs::CoverageResult result;

/**
 * @brief An action server type that allows to start and stop the coverage task.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::CoverageAction> action_server_t;

/**
 * @brief The random number generator used for calculating help tracking probability.
 */
random_numbers::RandomNumberGenerator* rng;

/**
 * @brief The distance in meter within which help calls of other CPSs are considered.
 */
double help_range;

/**
 * @brief Current position of the CPS.
 */
geometry_msgs::Pose pose;

/**
 * @brief Callback of the action server which executes the coverage task until it is preempted or finished.
 * @param goal The goal message received from the action client.
 * @param as The action server offered by this node.
 */
void ActionCallback(const cpswarm_msgs::CoverageGoalConstPtr& goal, action_server_t* as)
{
    NodeHandle nh;

    // set loop rate
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);

    ROS_INFO("Executing coverage");

    // coverage library
    uav_random_direction uav_coverage(goal->altitude);

    // execute coverage until state changes
    state = STATE_ACTIVE;
    while (ok() && !as->isPreemptRequested() && state == STATE_ACTIVE) {
        behavior_state_t result = uav_coverage.step();
        if (state == STATE_ACTIVE)
            state = result;
        rate.sleep();
        spinOnce();
    }

    // stop moving
    uav_coverage.stop();

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
        ROS_INFO("Coverage preempted");
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
 * @brief Callback function to receive help calls for tracking targets found by other CPSs.
 * @param msg ID, position, and tracking times of the target.
 */
void help_callback (const cpswarm_msgs::TargetHelp::ConstPtr& msg)
{
    // distance to target
    double dist = hypot(msg->pose.pose.position.x - pose.position.x, msg->pose.pose.position.y - pose.position.y);

    // relative tracking time
    double time;
    if (msg->time_need > 0)
        time = msg->time_avail / msg->time_need;
    else
        time = 1;

    // probability for helping
    double p = exp(dist * 2 * log(0.5) / help_range * time);
    double rand = rng->uniform01();
    if (rand < p)
        state = STATE_PREEMPTED;
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

    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // stop covering once a target has been found
    bool single_target;
    nh.param(this_node::getName() + "/single_target", single_target, true);
    Subscriber found_sub;
    if (single_target)
        found_sub = nh.subscribe("target_found", queue_size, found_callback);

    // help other cpss with tracking
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

