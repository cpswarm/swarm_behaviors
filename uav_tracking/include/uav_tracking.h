#ifndef UAV_TRACKING_H
#define UAV_TRACKING_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <random_numbers/random_numbers.h>
#include <cpswarm_msgs/TrackingAction.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include <cpswarm_msgs/TargetTrackedBy.h>
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

/**
 * @brief The random number generator used for calculating probability to stop tracking.
 */
random_numbers::RandomNumberGenerator* rng;

/**
 * @brief The number of UAVs that are currently tracking the same target.
 */
int trackers;

/**
 * @brief The maximum number of UAVs that should be tracking the same target simultaneously.
 */
int max_trackers;

/**
 * @brief The number of UAVs tracking the same target above which a UAV stops tracking with a certain probability.
 */
int min_trackers;

#endif // UAV_TRACKING_H