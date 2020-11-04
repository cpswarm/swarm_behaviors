#ifndef UAV_FLOCKING_H
#define UAV_FLOCKING_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <cpswarm_msgs/ArrayOfVectors.h>
#include <cpswarm_msgs/ArrayOfPositions.h>
#include "cpswarm_msgs/GetPoints.h"
#include "position.h"
#include "velocity.h"

/**
 * @brief A behavior library that allows to perform flocking with a swarm of unmanned areal vehicles (UAVs).
 */
class uav_flocking
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param altitude: The altitude at which the CPS operates.
     */
    uav_flocking (double altitude);

    /**
     * @brief Get the center of the swarm.
     * @return The arithmetic mean of all UAV positions.
     */
    geometry_msgs::Point center ();

    /**
     * @brief Compute the velocity to perform coverage with a flock of UAVs.
     * @param velocity The desired velocity of the flock members in order to perform coverage.
     * @return The velocity adjusted for the UAVs to stay in the flock.
     */
    geometry_msgs::Vector3 coverage (geometry_msgs::Vector3 velocity);

    /**
     * @brief Compute the velocity to perform tracking with a flock of UAVs.
     * @param target The position of the target to track.
     * @return The velocity to track the target while staying in the desired formation.
     */
    geometry_msgs::Vector3 tracking (geometry_msgs::Pose target);

private:
    /**
     * @brief Compute acceleration to align velocities between UAVs.
     */
    void alignment ();

    /**
     * @brief Compute the distance of the area bound to the coordinate system origin. A rectangular area is assumed.
     * @return The distance between origin and the point on the area bound where the line from origin through the current UAV pose intersects.
     */
    double dist_bound();

    /**
     * @brief Compute velocity that allows UAVs to stay in the flock.
     */
    void flocking (geometry_msgs::Vector3 vel);

    /**
     * @brief Compute velocity that allows UAVs to stay in the desired formation.
     * @todo Finish implementation. TODO
     */
    void formation (geometry_msgs::Point target);

    /**
     * @brief Get the relative velocity of a specific UAV.
     * @param uuid The UUID of the UAV to get the velocity for.
     * @return The magnitude and direction of relative velocity of given CPS. An empty vector in case the UUID is unknown.
     */
    cpswarm_msgs::Vector get_velocity (string uuid) const;

    /**
     * @brief Compute acceleration from repulsive forces between UAVs.
     */
    void repulsion ();

    /**
     * @brief A transfer function to smooth the accelerations and velocities.
     * @param x The x value of the transfer function, e.g., position of UAV.
     * @param r The x value below which the transfer function is zero.
     * @param d The width of x values in which the transfer function is active. For values above r+d, the transfer function is one, i.e., maximal.
     * @return A value between zero and one which transfers smoothly using the sine function.
     */
    double transfer (double x, double r, double d);

    /**
     * @brief Compute acceleration from repulsive forces of bounding virtual walls, i.e., environment boundaries.
     */
    void wall ();

    /**
     * @brief Callback function to receive the relative positions of the other CPSs.
     * @param msg Distance and bearing of the other CPSs.
     */
    void swarm_pose_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg);

    /**
     * @brief Callback function to receive the absolute positions of the other CPSs.
     * @param msg Coordinates of the other CPSs.
     */
    void swarm_pose_abs_callback (const cpswarm_msgs::ArrayOfPositions::ConstPtr& msg);

    /**
     * @brief Callback function to receive the relative velocities of the other CPSs.
     * @param msg Distance and bearing of the other CPSs.
     */
    void swarm_vel_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg);

    /**
     * @brief Subscriber for the relative positions of the other CPSs.
     */
    Subscriber swarm_pose_sub;

    /**
     * @brief Subscriber for the absolute positions of the other CPSs.
     */
    Subscriber swarm_pose_abs_sub;

    /**
     * @brief Subscriber for the relative velocities of the other CPSs.
     */
    Subscriber swarm_vel_sub;

    /**
     * @brief Service client to get mission area boundaries.
     */
    ServiceClient area_client;

    /**
     * @brief The type of formation the UAVs should take on.
     */
    string form;

    /**
     * @brief A helper object for position related tasks.
     */
    position pos;

    /**
     * @brief A helper object for velocity related tasks.
     */
    velocity vel;

    /**
     * @brief The relative positions of the other swarm members (distance and bearing).
     */
    vector<cpswarm_msgs::VectorStamped> swarm_pos;

    /**
     * @brief The absolute positions of the other swarm members.
     */
    vector<cpswarm_msgs::Position> swarm_pos_abs;

    /**
     * @brief The relative velocities of the other swarm members (distance and bearing).
     */
    vector<cpswarm_msgs::VectorStamped> swarm_vel;

    /**
     * @brief The period length of one control loop.
     */
    double dt;

    /**
     * @brief Acceleration from repulsive forces between UAVs.
     */
    geometry_msgs::Vector3 a_repulsion;

    /**
     * @brief Acceleration to align velocities between UAVs.
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
     * @brief Equilibrium distance between UAVs.
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
     * @brief Maximum repulsion between UAVs.
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
     * @brief Minimum alignment between UAVs.
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
     * @brief Softness of shape.
     */
    double form_decay;

    /**
     * @brief Characteristic time needed by the UAV to reach the target velocity.
     */
    double accel_time;
};

#endif // UAV_FLOCKING_H

