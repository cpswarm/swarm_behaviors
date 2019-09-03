#ifndef UAV_FLOCKING_TRACKING_H
#define UAV_FLOCKING_TRACKING_H

#include "uav_tracking.h"
#include "uav_flocking.h"

/**
 * @brief An implementation of the tracking class that allows to track a target by minimizing the offset between the cyber physical system (CPS) and the target.
 */
class uav_flocking_tracking : public uav_tracking
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param target The target being tracked by this UAV.
     * @param ugv The UGV that has been assigned to rescue the tracked target.
     */
    uav_flocking_tracking (unsigned int target, string ugv);

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
     * @brief The flocking behavior library object.
     */
    uav_flocking* flock;
};

#endif // UAV_FLOCKING_TRACKING_H
