#ifndef UAV_TRACKING_H
#define UAV_TRACKING_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cpswarm_msgs/TrackingAction.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include "lib/uav_flocking_tracking.h"
#include "lib/uav_simple_tracking.h"

/**
 * @brief An action server type that allows to start and stop the tracking task.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::TrackingAction> action_server_t;

/**
 * @brief The type of coverage to perform.
 */
string behavior;

/**
 * @brief The state of the behavior algorithm.
 */
behavior_state_t state;

/**
 * @brief The target being tracked.
 */
cpswarm_msgs::TargetPositionEvent target;

#endif // UAV_TRACKING_H