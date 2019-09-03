#ifndef TARGETS_H
#define TARGETS_H

#include "position.h"
#include "target.h"
#include "cpswarm_msgs/TargetTracking.h"

/**
 * @brief A collection of target objects that are searched, tracked, and rescued by the CPSs.
 */
class targets
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param cps The UUID of the CPS that owns this class instance.
     */
    targets (string cps);

    /**
     * @brief Publish a target position event for the given target.
     * @param id The ID of the target.
     */
    void publish (unsigned int id);

    /**
     * @brief Check whether the CPS owning this class instance has rescued a target.
     * @return True, if the CPS has rescued any of the known targets, false otherwise.
     */
    bool rescued () const;

    /**
     * @brief Check whether the given target has been rescued.
     * @param id The ID of the target to check.
     * @return True, if the given target has been rescued, false otherwise.
     */
    bool rescued (unsigned int id);

    /**
     * @brief Check whether the CPS owning this class instance is tracking a target.
     * @param target Returns the details of the target being tracked.
     * @return True, if the CPS is tracking a target, false otherwise.
     */
    bool tracking (target& target) const;

    /**
     * @brief Check whether the CPS owning this class instance is tracking the given target.
     * @param id The ID of the target to check.
     * @return True, if the CPS is tracking the given target, false otherwise.
     */
    bool tracking (unsigned int id);

    /**
     * @brief Update the state of all targets. If no update has been received for a target within the tracking timeout, its state will change to TARGET_LOST. This needs to be called periodically.
     */
    void update () const;

    /**
     * @brief Update the information of a target.
     * @param msg The target position event message.
     * @param state The target state.
     * @param guided_by The UUID of the CPS that is guiding the target.
     * @param tracked_by The UUID if the CPS that is tracking the target.
     *
     */
    void update(cpswarm_msgs::TargetPositionEvent msg, target_state_t state, string guided_by="", string tracked_by="");

private:
    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Publisher for transmitting target tracking information.
     */
    Publisher tracking_pub;

    /**
     * @brief A helper object for position related tasks.
     */
    position pos;

    /**
     * @brief A map holding ID and target object of all known targets.
     */
    map<unsigned int, target> target_map;

    /**
     * @brief A map holding ID and target object of all possible targets, including the ones not yet found.
     */
    map<unsigned int, target> target_map_all;

    /**
     * @brief The UUID of the CPS that owns this class instance.
     */
    string cps;

    /**
     * @brief The field of view of the target tracking camera in radian (measured). It is used to simulate target detection.
     */
    double fov;
};

#endif // TARGETS_H
