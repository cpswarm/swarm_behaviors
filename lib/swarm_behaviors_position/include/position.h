#ifndef POSITION_H
#define POSITION_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/Pose.h>
#include <cpswarm_msgs/OutOfBounds.h>
#include <cpswarm_msgs/GetSector.h>

using namespace std;
using namespace ros;

/**
 * @brief A class to provide position related functionalities.
 */
class position
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     * @param altitude The altitude at which the CPS operates.
     */
    position (double altitude);

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~position ();

    /**
     * @brief Compute the bearing from the current pose of the CPS to a given pose.
     * @param p The pose to compute bearing of.
     * @return The bearing of the given pose relative to the current yaw of the CPS, counterclockwise.
     */
    double bearing (geometry_msgs::Pose p) const;

    /**
     * @brief Compute the goal coordinates relative to the current position.
     * @param distance The distance of the goal from the current position.
     * @param direction The direction of the goal relative to the current position. It is in radian, counterclockwise starting from east / x-axis.
     * @return The goal pose.
     */
    geometry_msgs::Pose compute_goal (double distance, double direction) const;

    /**
     * @brief Compute the goal coordinates relative to a given start position.
     * @param start The starting position.
     * @param distance The distance of the goal from start.
     * @param direction The direction of the goal relative to start. It is in radian, counterclockwise starting from east / x-axis.
     * @return The goal pose.
     */
    geometry_msgs::Pose compute_goal (geometry_msgs::Pose start, double distance, double direction) const;

    /**
     * @brief Compute the straight-line distance from the current position to the given position.
     * @param p The pose to compute distance to.
     * @return The distance in meters.
     */
    double dist (geometry_msgs::Pose p) const;

    /**
     * @brief Compute the straight-line distance between two positions.
     * @param p1 First pose.
     * @param p2 Second pose.
     * @return The distance in meters.
     */
    double dist (geometry_msgs::Pose p1, geometry_msgs::Pose p2) const;

    /**
     * @brief Get the current pose of the CPS.
     * @return The current pose of the CPS in local coordinates.
     */
    geometry_msgs::Pose get_pose () const;

    /**
     * @brief Get the position tolerance.
     * @return The distance in meter that the CPS can be away from a goal while still being considered to have reached that goal.
     */
    double get_tolerance () const;

    /**
     * @brief Get the current yaw orientation of the CPS.
     * @return The current yaw angle of the CPS counterclockwise starting from x-axis/east.
     */
    double get_yaw () const;

    /**
     * @brief Move the CPS to the given pose.
     * @param pose The position to move to.
     * @return Whether the CPS reached the goal position.
     */
    bool move (geometry_msgs::Pose pose);

    /**
     * @brief Check whether there is an obstacle in the direction of the given pose.
     * @param pose The pose to check.
     * @return True, if there is an obstacle in the direction of the pose, false otherwise.
     */
    bool occupied (geometry_msgs::Pose pose);

    /**
     * @brief Check whether a given pose is out of the mission area boundaries.
     * @param pose The pose to check.
     * @return True, if the given pose is outside the mission area or it could not be checked, false otherwise.
     */
    bool out_of_bounds (geometry_msgs::Pose pose);

    /**
     * @brief Check whether the CPS has reached the current goal.
     * @param goal An optional goal to check instead of the class variable. Make sure time stamp is set.
     * @return True if the CPS is close to the current goal, false otherwise.
     */
    bool reached (geometry_msgs::PoseStamped goal = geometry_msgs::PoseStamped());

    /**
     * @brief Stop moving by publishing the current position as goal.
     */
    void stop ();

private:
    /**
     * @brief Get the yaw orientation from a pose.
     * @param pose The pose that contains the orientation.
     * @return The yaw angle of the given pose counterclockwise starting from x-axis/east.
     */
    double get_yaw (geometry_msgs::Pose pose) const;

    /**
     * @brief Callback function for position updates.
     * @param msg Position received from the CPS.
     */
    void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief Subscriber for the position of the CPS.
     */
    Subscriber pose_sub;

    /**
     * @brief Service client for determining whether the goal is out of the area bounds.
     */
    ServiceClient out_of_bounds_client;

    /**
     * @brief Service client for determining the sector occupied by obstacles.
     */
    ServiceClient occupied_sector_client;

    /**
     * @brief Publisher for sending the goal position of the CPS to the position controller in the abstraction library.
     */
    Publisher pose_pub;

    /**
     * @brief Publisher to visualize the current goal.
     */
    Publisher visualize_pub;

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief The loop rate object for running the behavior control loops at a specific frequency.
     */
    Rate* rate;

    /**
     * @brief Current position of the CPS.
     */
    geometry_msgs::Pose pose;

    /**
     * @brief Current goal of the CPS.
     */
    geometry_msgs::PoseStamped goal;

    /**
     * @brief Whether a valid position has been received.
     */
    bool pose_valid;

    /**
     * @brief The distance that the CPS can be away from a goal while still being considered to have reached that goal.
     */
    double goal_tolerance;

    /**
     * @brief The time in seconds that reaching a waypoint is allowed to take.
     */
    Duration move_timeout;

    /**
     * @brief Whether to publish the goal waypoint on a topic for visualization.
     */
    bool visualize;

    /**
     * @brief The altitude at which the CPS operates.
     */
    double altitude;
};

#endif // POSITION_H
