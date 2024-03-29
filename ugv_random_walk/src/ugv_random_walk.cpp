#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cpswarm_msgs/CoverageAction.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include "lib/ugv_random_walk.h"

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
    ugv_random_walk ugv_coverage;

    // execute coverage until state changes
    state = STATE_ACTIVE;
    while (ok() && !as->isPreemptRequested() && state == STATE_ACTIVE) {
        ROS_DEBUG("Coverage step");
        behavior_state_t result = ugv_coverage.step();
        if (state == STATE_ACTIVE)
            state = result;
        rate.sleep();
        spinOnce();
    }

    // stop moving
    ugv_coverage.stop();

    // coverage succeeded
    if (state == STATE_SUCCEEDED) {
        ROS_INFO("Coverage succeeded, found target %s at [%f, %f]", result.target_id.c_str(), result.target_pose.pose.position.x, result.target_pose.pose.position.y);

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
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main (int argc, char** argv)
{
    // init ros node
    init(argc, argv, "ugv_coverage");
    NodeHandle nh;

    // stop covering once a target has been found
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    bool single_target;
    nh.param(this_node::getName() + "/single_target", single_target, true);
    Subscriber found_sub;
    if (single_target)
        found_sub = nh.subscribe("target_found", queue_size, found_callback);

    // start action server
    action_server_t as(nh, "ugv_coverage", boost::bind(&ActionCallback, _1, &as), false);
    as.start();

    // wait for action client
    spin();

    return 0;
}

