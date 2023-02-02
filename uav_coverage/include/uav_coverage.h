#ifndef UAV_COVERAGE_H
#define UAV_COVERAGE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <cpswarm_msgs/CoverageAction.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include <cpswarm_msgs/TargetHelp.h>
#include "lib/uav_flocking_coverage.h"
#include "lib/uav_spiral_coverage.h"
#include "lib/uav_random_coverage.h"
#include "lib/uav_systematic_coverage.h"

/**
 * @brief An action server type that allows to start and stop the coverage task.
 */
typedef actionlib::SimpleActionServer<cpswarm_msgs::CoverageAction> action_server_t;

/**
 * @brief Current position of the CPS.
 */
geometry_msgs::Pose pose;

/**
 * @brief The type of coverage to perform.
 */
string behavior;

/**
 * @brief The state of the behavior algorithm.
 */
behavior_state_t state;

/**
 * @brief The coverage result, i.e., target ID and position.
 */
cpswarm_msgs::CoverageResult result;

/**
 * @brief The random number generator used for calculating help probability.
 */
random_numbers::RandomNumberGenerator* rng;

/**
 * @brief The distance in meter within which help calls of other CPSs are considered.
 */
double help_range_max;

/**
 * @brief The distance in meter below which help calls of other CPSs are answered for sure.
 */
double help_range_min;

#endif // UAV_COVERAGE_H
