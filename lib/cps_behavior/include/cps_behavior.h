#ifndef CPS_BEHAVIOR_H
#define CPS_BEHAVIOR_H

#include <unordered_map>
#include <queue>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include <cpswarm_msgs/TargetTracking.h>
#include <cpswarm_msgs/TargetAssignmentEvent.h>
#include <swarmros/String.h>
#include "position.h"
#include "velocity.h"
#include "targets.h"

using namespace std;
using namespace ros;

/**
 * @brief An enumeration for the state of the behavior algorithm.
 */
typedef enum {
    STATE_ACTIVE = 0,
    STATE_SUCCEEDED,
    STATE_ABORTED
} behavior_state_t;

/**
 * @brief An abstract base class for all behaviors of cyber physical systems (CPSs).
 */
class cps_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    cps_behavior ();

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~cps_behavior ();

    /**
     * @brief Execute one cycle of the behavior algorithm.
     * @return Return the state of the behavior algorithm.
     */
    virtual behavior_state_t step () = 0;

protected:
    /**
     * @brief Move the CPS to the given pose.
     * @param pose The position to move to.
     */
    void move (geometry_msgs::Pose pose);

    /**
     * @brief Move the CPS with the given velocity.
     * @param pose The velocity to move with.
     */
    void move (geometry_msgs::Vector3 velocity);

    /**
     * @brief Callback function for incoming target rescued events.
     * @param msg ID and last position of target.
     */
    virtual void target_rescued_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg);

    /**
     * @brief Publisher for sending the goal position of the CPS to the position controller in the abstraction library.
     */
    Publisher pose_pub;

    /**
     * @brief Publisher for sending the target velocity of the CPS to the velocity controller in the abstraction library.
     */
    Publisher velocity_pub;

    /**
     * @brief UUID of this CPS.
     */
    string id;

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief The state of the behavior algorithm.
     */
    behavior_state_t state;

    /**
     * @brief A helper object for position related tasks.
     */
    position* pos;

    /**
     * @brief A helper object for velocity related tasks.
     */
    velocity* vel;

    /**
     * @brief A collection of all targets that have been found so far.
     */
    targets* sar_targets;

    /**
     * @brief The loop rate object for running the behavior control loops at a specific frequency.
     */
    Rate* rate;

private:
    /**
     * @brief Callback function to receive information about targets found by other CPSs.
     * @param msg ID and position of target.
     */
    void target_found_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg);

    /**
     * @brief Callback function to receive information about targets lost by other CPSs.
     * @param msg ID and last known position of target.
     */
    void target_lost_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg) const;

    /**
     * @brief Callback function to receive the UUID from the communication library.
     * @param msg UUID of this node.
     */
    void uuid_callback (const swarmros::String::ConstPtr& msg);

    /**
     * @brief Subscriber for receiving information about targets found by other CPSs.
     */
    Subscriber target_found_sub;

    /**
     * @brief Subscriber for receiving information about targets lost by other CPSs.
     */
    Subscriber target_lost_sub;

    /**
     * @brief Subscriber for receiving information about targets rescued by other CPSs.
     */
    Subscriber target_rescued_sub;

    /**
     * @brief Subscriber for receiving the UUID from the communication library.
     */
    Subscriber uuid_sub;
};

#endif // CPS_BEHAVIOR_H
