#ifndef UAV_FLOCKING_COVERAGE_H
#define UAV_FLOCKING_COVERAGE_H

#include <position.h>
#include <velocity.h>
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
 * @brief An implementation of the coverage class that allows to cover a given area with the flocking algorithm.
 */
class uav_flocking_coverage
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    uav_flocking_coverage ();

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~uav_flocking_coverage ();

    /**
     * @brief Move the swarm member to a new position.
     * @return Return the state of the coverage algorithm.
     */
    behavior_state_t step ();

    /**
     * @brief Stop moving.
     */
    void stop ();

private:
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
     * @brief The path object to compute the boustrophedon coverage path.
     */
//     boustrophedon_path* path;

    /**
     * @brief Target velocity of the flock.
     */
    double flock_vel;

    /**
     * @brief Publisher to visualize waypoints.
     */
    Publisher pub_vis_wp;
};

#endif // UAV_FLOCKING_COVERAGE_H

