#ifndef UAV_FLOCKING_TRACKING_H
#define UAV_FLOCKING_TRACKING_H

#include <position.h>
#include <cpswarm_msgs/TargetPositionEvent.h>
#include "uav_flocking.h"

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
 * @brief An implementation of the tracking class that allows to track a target by minimizing the offset between the cyber physical system (CPS) and the target.
 */
class uav_flocking_tracking
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param target The ID of the target being tracked by this UAV.
     */
    uav_flocking_tracking (unsigned int target);

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~uav_flocking_tracking ();

    /**
     * @brief Execute one cycle of the tracking algorithm.
     * @return Return the state of the tracking algorithm.
     */
    behavior_state_t step ();

private:
    /**
     * @brief Callback function to receive target position updates.
     * @param msg ID and position of target.
     */
    void target_callback (const cpswarm_msgs::TargetPositionEvent::ConstPtr& msg);

    /**
     * @brief Subscriber to receive target position updates.
     */
    Subscriber target_sub;

    /**
     * @brief A helper object for position related tasks.
     */
    position pos;

    /**
     * @brief A helper object for velocity related tasks.
     */
    velocity vel;

    /**
     * @brief The flocking behavior library object.
     */
    uav_flocking* flock;

    /**
     * @brief The target being tracked by this UAV.
     */
    cpswarm_msgs::TargetPositionEvent target;
};

#endif // UAV_FLOCKING_TRACKING_H
