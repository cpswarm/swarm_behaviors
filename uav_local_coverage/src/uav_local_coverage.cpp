#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <cpswarm_msgs/CoverageAction.h>
#include "lib/uav_local_coverage.h"

using namespace ros;

/**
 * @brief An action server type that allows to start and stop the local coverage task.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::CoverageAction> action_server_t;

/**
 * @brief Callback of the action server which executes the local coverage task until it is preempted or finished.
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

    ROS_INFO("Executing local coverage");

    // coverage library
    uav_local_coverage* uav_coverage = new uav_local_coverage();

    // found target id and pose
    cpswarm_msgs::CoverageResult result;

    // execute coverage until state changes
    behavior_state_t state = STATE_ACTIVE;
    while (ok() && !as->isPreemptRequested() && state == STATE_ACTIVE) {
        ROS_INFO("Local coverage step");
        state = uav_coverage->step();
        rate.sleep();
    }

    // coverage succeeded
    if (state == STATE_SUCCEEDED) {
        ROS_INFO("Local coverage succeeded");

        // get target information
        if (uav_coverage->get_target(result))
            ROS_INFO("Found target %d at [%f, %f]", result.target_id, result.target_pose.position.x, result.target_pose.position.y);
        else
            ROS_INFO("No target found");

        as->setSucceeded();
    }

    // coverage aborted
    else if (state == STATE_ABORTED) {
        ROS_INFO("Local coverage aborted");
        as->setAborted();
    }

    // coverage was preempted
    else{
        ROS_INFO("Local coverage preempted");
        as->setPreempted();
    }

    // destroy coverage library
    delete uav_coverage;
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
    init(argc, argv, "cpswarm_sar_uav_local_coverage");
    NodeHandle nh;

    // define which log messages are shown
    if (console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console::levels::Info)) {
        console::notifyLoggerLevelsChanged();
    }
    else{
        ROS_ERROR("Could not set logger level!");
    }

    // start action server
    action_server_t as(nh, "uav_local_coverage", boost::bind(&ActionCallback, _1, &as), false);
    as.start();

    // wait for action client
    spin();

    return 0;
}

