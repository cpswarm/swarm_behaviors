#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include <cpswarm_msgs/TrackingAction.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include "lib/uav_simple_tracking.h"

using namespace ros;

/**
 * @brief The state of the behavior algorithm.
 */
behavior_state_t state;

/**
 * @brief The ID of the target being tracked.
 */
int target;

/**
 * @brief An action server type that allows to start and stop the tracking task.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::TrackingAction> action_server_t;

/**
 * @brief Callback of the action server which executes the tracking task until it is preempted or finished.
 * @param goal The goal message received from the action client.
 * @param as The action server offered by this node.
 */
void ActionCallback(const cpswarm_msgs::TrackingGoalConstPtr& goal, action_server_t* as)
{
    NodeHandle nh;

    // target id
    target = goal->target;

    // set loop rate
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);

    ROS_INFO("Executing tracking");

    // tracking library
    uav_simple_tracking uav_tracking(target, goal->altitude);

    // execute tracking until state changes
    behavior_state_t state = STATE_ACTIVE;
    while (ok() && !as->isPreemptRequested() && state == STATE_ACTIVE) {
        ROS_DEBUG("Tracking step");
        behavior_state_t result = uav_tracking.step();
        if (state == STATE_ACTIVE)
            state = result;
        rate.sleep();
        spinOnce();
    }

    // stop moving
    uav_tracking.stop();

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
 * @brief Callback function to receive event that target has been lost.
 * @param msg ID and position of target.
 */
void lost_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    if (msg->id == target)
        state = STATE_ABORTED;
}

/**
 * @brief Callback function to receive event that target has been done.
 * @param msg ID and position of target.
 */
void done_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg)
{
    if (msg->id == target)
        state = STATE_SUCCEEDED;
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

    // define which log messages are shown
    if (console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console::levels::Info)) {
        console::notifyLoggerLevelsChanged();
    }
    else{
        ROS_ERROR("Could not set logger level!");
    }

    // initially, no targets being tracked
    target = -1;

    // subscribers
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    Subscriber lost_sub = nh.subscribe("target_lost", queue_size, lost_callback);
    Subscriber done_sub = nh.subscribe("target_done", queue_size, done_callback);

    // start action server
    action_server_t as(nh, "uav_tracking", boost::bind(&ActionCallback, _1, &as), false);
    as.start();

    // wait for action client
    spin();

    return 0;
}

