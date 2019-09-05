#ifndef UGV_RANDOM_WALK_H
#define UGV_RANDOM_WALK_H

#include <random_numbers/random_numbers.h>
#include "cpswarm_msgs/closest_bound.h"
#include "cpswarm_msgs/get_sector.h"
#include "ugv_coverage.h"

/**
 * @brief An implementation of the coverage class that allows to cover a given area with the random walk algorithm. The random walk is a mathematical movement model, where an agent moves straight for a specific distance and then changes its direction randomly.
 */
class ugv_random_walk : public ugv_coverage
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param seed Seed for random number generation. If not set, a random seed will be used.
     */
    ugv_random_walk (int seed=0);

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~ugv_random_walk ();

    /**
     * @brief Move the swarm member to a new position.
     * @return Return the state of the coverage algorithm.
     */
    behavior_state_t step();

private:
    /**
     * @brief Compute new direction using rng.
     */
    void new_direction ();

    /**
     * @brief Compute new direction as reflection from wall.
     */
    void reflect ();

    /**
     * @brief Compute goal position from direction.
     * @return The selected goal.
     */
    geometry_msgs::Pose select_goal ();

    /**
     * @brief Service client for determining closest mission area boundary to the current UGV position.
     */
    ServiceClient bound_client;

    /**
     * @brief Service client for determining the sector clear of obstacles.
     */
    ServiceClient clear_sector_client;

    /**
     * @brief Whether the UGV still needs to turn before moving.
     */
    bool turn;

    /**
     * @brief The direction in which the UGV is traveling. It is measured in radian, clockwise starting from north.
     */
    double direction;

    /**
     * @brief The distance which the UGV is traveling in each step.
     */
    double distance;

    /**
     * @brief The random number generator used for selecting a random direction.
     */
    random_numbers::RandomNumberGenerator* rng;
};

#endif // UGV_RANDOM_WALK_H

