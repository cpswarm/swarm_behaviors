#ifndef TARGET_H
#define TARGET_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "cpswarm_msgs/TargetPositionEvent.h"

using namespace std;
using namespace ros;

/**
 * @brief An enumeration for the state of a target.
 */
typedef enum {
    TARGET_UNKNOWN = 0,
    TARGET_TRACKED,
    TARGET_RESCUED,
    TARGET_LOST
} target_state_t;

/**
 * @brief A target that is searched, tracked, and rescued by the CPSs.
 */
class target
{
public:
    /**
     * @brief Constructor.
     */
    target ();

    /**
     * @brief Constructor that initializes some private member variables.
     * @param id The target ID.
     * @param state The target state.
     */
    target (unsigned int id, target_state_t state);

    /**
     * @brief Constructor that initializes some private member variables.
     * @param id The target ID.
     * @param state The target state.
     * @param pose The position of the target.
     */
    target (unsigned int id, target_state_t state, geometry_msgs::Pose pose);

    /**
     * @brief Constructor that initializes some private member variables.
     * @param id The target ID.
     * @param state The target state.
     * @param pose The position of the target.
     * @param stamp The time stamp of target.
     */
    target (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp);

    /**
     * @brief Constructor that initializes all private member variables.
     * @param id The target ID.
     * @param state The target state.
     * @param pose The position of the target.
     * @param stamp The time stamp of the target.
     * @param guided_by The UUID of the CPS that is guiding the target.
     * @param tracked_by The UUID if the CPS that is tracking the target.
     */
    target (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp, string guided_by, string tracked_by);

    /**
     * @brief Get the ID of the target.
     * @return The ID of the target. Negative IDs are invalid.
     */
    int get_id () const;

    /**
     * @brief Get the position of the target.
     * @return The position of the target.
     */
    geometry_msgs::Pose get_pose () const;

    /**
     * @brief Check whether the target has been lost. Set the state to TARGET_LOST if the tracking timeout has expired.
     * @param cps The UUID of the CPS that was last tracking the target.
     * @return True if the target is in the state TARGET_LOST or if no update has been received within tracking timeout, false otherwise.
     */
    bool lost (string cps);

    /**
     * @brief Publish a target position event about the target.
     */
    void publish ();

    /**
     * @brief Check whether the target has been rescued.
     * @return True, if the target is in the state TARGET_RESCUED, false otherwise.
     */
    bool rescued () const;

    /**
     * @brief Check whether the target has been rescued.
     * @param cps The UUID of the CPS that was last tracking the target.
     * @return True, if the target is in the state TARGET_RESCUED and the given CPS was last tracking it.
     */
    bool rescued (string cps) const;

    /**
     * @brief Check whether the target is being tracked by the given CPS.
     * @param cps The UUID of the CPS that is supposed to track the target.
     * @return True, if the target is tracked by the given CPS, false otherwise.
     */
    bool tracked (string cps) const;

    /**
     * @brief Update the information about a target.
     * @param id The ID of the target.
     * @param state The state of the target.
     * @param pose The position of the target.
     * @param stamp The time stamp of the update.
     * @param guided_by The CPS that is guiding the target.
     * @param tracked_by The CPS that is tracking the target.
     */
    void update (unsigned int id, target_state_t state, geometry_msgs::Pose pose, Time stamp, string guided_by, string tracked_by);

private:
    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief The ID of this target. Negative IDs are invalid.
     */
    int id;

    /**
     * @brief State of the target.
     */
    target_state_t state;

    /**
     * @brief Position of the target.
     */
    geometry_msgs::Pose pose;

    /**
     * @brief The position of the target the last time it was published as target position event.
     */
    geometry_msgs::Pose last_pose;

    /**
     * @brief Time stamp of latest update of the target.
     */
    Time stamp;

    /**
     * @brief The UUID of the CPS that is guiding the target.
     */
    string guided_by;

    /**
     * @brief A list of the UUIDs of the CPSs that are tracking the target.
     */
    set<string> tracked_by;

    /**
     * @brief The time in seconds after which a target transistions into the state TARGET_LOST when no target update has been received.
     */
    Duration tracking_timeout;

    /**
     * @brief The loop rate object for running the behavior control loops at a specific frequency.
     */
    Rate* rate;

    /**
     * @brief Minimum distance between two consecutive target positions such that a target update event is published.
     */
    double target_tolerance;

    /**
     * @brief Publisher for transmitting information about targets found to other CPSs.
     */
    Publisher target_found_pub;

    /**
     * @brief Publisher for transmitting updated information about targets to other CPSs.
     */
    Publisher target_update_pub;

    /**
     * @brief Publisher for transmitting information about targets lost to other CPSs.
     */
    Publisher target_lost_pub;

    /**
     * @brief The name of the target found event.
     */
    string target_found_event;

    /**
     * @brief The name of the target lost event.
     */
    string target_lost_event;

    /**
     * @brief The name of the target update event.
     */
    string target_update_event;
};

#endif // TARGET_H
