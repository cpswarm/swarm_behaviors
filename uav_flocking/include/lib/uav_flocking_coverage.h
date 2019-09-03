#ifndef UAV_FLOCKING_COVERAGE_H
#define UAV_FLOCKING_COVERAGE_H

#include "uav_coverage.h"
#include "uav_flocking.h"
#include "boustrophedon_path.h"

/**
 * @brief An implementation of the coverage class that allows to cover a given area with the flocking algorithm.
 */
class uav_flocking_coverage : public uav_coverage
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

private:
    /**
     * @brief The flocking behavior library object.
     */
    uav_flocking* flock;

    /**
     * @brief The path object to compute the boustrophedon coverage path.
     */
    boustrophedon_path* path;

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

