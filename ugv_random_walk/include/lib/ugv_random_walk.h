#ifndef UGV_RANDOM_WALK_H
#define UGV_RANDOM_WALK_H

#include <random_numbers/random_numbers.h>
#include "cpswarm_msgs/ClosestBound.h"
#include "cpswarm_msgs/GetSector.h"
#include "position.h"

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
 * @brief An implementation of the coverage class that allows to cover a given area with the random walk algorithm. The random walk is a mathematical movement model, where an agent moves straight for a specific distance and then changes its direction randomly.
 */
class ugv_random_walk
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    ugv_random_walk ();

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
     * @return Whether a a new direction could be set successfully.
     */
    bool new_direction ();

    /**
     * @brief Compute new direction as reflection from wall.
     * @return Whether a a new direction could be set successfully.
     */
    bool reflect ();

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
     * @brief A helper object for position related tasks.
     */
    position pos;

    /**
     * @brief The distance that the UGV travels in one step.
     */
    double step_size;

    /**
     * @brief The direction in which the UGV is traveling. It is measured in radian, clockwise starting from north.
     */
    double direction;

    /**
     * @brief The random number generator used for selecting a random direction.
     */
    random_numbers::RandomNumberGenerator* rng;
};

#endif // UGV_RANDOM_WALK_H

