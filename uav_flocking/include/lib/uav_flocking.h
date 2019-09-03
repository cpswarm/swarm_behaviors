#ifndef UAV_FLOCKING_H
#define UAV_FLOCKING_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include "cpswarm_msgs/closest_bound.h"
#include "swarm_position.h"
#include "swarm_velocity.h"
#include "position.h"
#include "velocity.h"

/**
 * @brief An enumeration for the formation of the flock used for tracking.
 */
typedef enum {
    FORM_GRID = 0,
    FORM_RING,
    FORM_LINE
} formation_t;

/**
 * @brief A behavior library that allows to perform flocking with a swarm of UAVs.
 */
class uav_flocking
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    uav_flocking ();

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~uav_flocking ();

    /**
     * @brief
     * @param
     * @return
     */
    geometry_msgs::Vector3 coverage (geometry_msgs::Vector3 velocity);

    /**
     * @brief
     * @param
     * @return
     */
    geometry_msgs::Vector3 tracking (geometry_msgs::Pose target);

private:
    /**
     * @brief Compute acceleration to align velocities between CPSs.
     */
    void alignment ();

    /**
     * TODO
     */
    void flocking (geometry_msgs::Vector3 vel);

    /**
     * TODO
     */
    void formation (geometry_msgs::Point target);

    /**
     * @brief Compute acceleration from repulsive forces between CPSs.
     */
    void repulsion ();

    /**
     * @brief TODO
     * @param x
     * @param r
     * @param d
     * @return
     */
    double transfer (double x, double r, double d);

    /**
     * @brief Compute acceleration from repulsive forces of bounding virtual walls.
     */
    void wall ();

    /**
     * @brief Service client for determining closest mission area boundary to the current UAV position.
     */
    ServiceClient bound_client;

    /**
     * @brief TODO
     */
    formation_t form;

    /**
     * @brief TODO
     */
    position pos;

    /**
     * @brief TODO
     */
    velocity vel;

    /**
     * TODO
     */
    swarm_position swarm_pos;

    /**
     * TODO
     */
    swarm_velocity swarm_vel;

    /**
     * @brief TODO
     */
    double dt;

    /**
     * @brief Acceleration from repulsive forces between CPSs.
     */
    geometry_msgs::Vector3 a_repulsion;

    /**
     * @brief Acceleration to align velocities between CPSs.
     */
    geometry_msgs::Vector3 a_alignment;

    /**
     * @brief Desired velocity of flock.
     */
    geometry_msgs::Vector3 v_flock;

    /**
     * @brief Acceleration to keep flock within area.
     */
    geometry_msgs::Vector3 a_wall;

    /**
     * @brief Velocity for formation flight.
     */
    geometry_msgs::Vector3 v_formation;

    /**
     * @brief Equilibrium distance between CPSs.
     */
    double equi_dist;

    /**
     * @brief Target velocity of the flock.
     */
    double flock_vel;

    /**
     * @brief Maximum velocity during formation flight.
     */
    double form_vel;

    /**
     * @brief Repulsion spring constant of half-spring.
     */
    double repulse_spring;

    /**
     * @brief Maximum repulsion between CPSs.
     */
    double repulse_max;

    /**
     * @brief Velocity alignment viscous friction coefficient.
     */
    double align_frict;

    /**
     * @brief Constant slope around equilibrium distance.
     */
    double align_slope;

    /**
     * @brief Minimum alignment between CPS.
     */
    double align_min;

    /**
     * @brief Bounding area viscous friction coefficient.
     */
    double wall_frict;

    /**
     * @brief Softness of wall as decay width.
     */
    double wall_decay;

    /**
     * @brief Strength of the shape forming velocity component in formation flight.
     */
    double form_shape;

    /**
     * @brief Strength of the tracking velocity component in formation flight.
     */
    double form_track;

    /**
     * @brief Characteristic time needed by the CPS to reach the target velocity.
     */
    double accel_time;
};

#endif // UAV_FLOCKING_H

