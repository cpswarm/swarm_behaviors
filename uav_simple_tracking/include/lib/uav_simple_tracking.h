#ifndef UAV_SIMPLE_TRACKING_H
#define UAV_SIMPLE_TRACKING_H

#include <position.h>
#include <cpswarm_msgs/TargetPositionEvent.h>

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
 * @brief An implementation that allows to track a target by minimizing the offset between the cyber physical system (CPS) and the target.
 */
class uav_simple_tracking
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param target The ID of the target being tracked by this UAV.
     */
    uav_simple_tracking (unsigned int target);

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
     * @brief The target being tracked by this UAV.
     */
    cpswarm_msgs::TargetPositionEvent target;
};

#endif // UAV_SIMPLE_TRACKING_H
