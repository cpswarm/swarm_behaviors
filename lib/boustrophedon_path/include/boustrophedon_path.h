#ifndef BOUSTROPHEDON_PATH_H
#define BOUSTROPHEDON_PATH_H

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "cpswarm_msgs/closest_bound.h"
#include "cpswarm_msgs/get_area.h"

using namespace std;
using namespace ros;

/**
 * @brief An class to compute a boustrophedon path of a rectangular area.
 */
class boustrophedon_path
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    boustrophedon_path ();

    /**
     * @brief Destructor that deletes the private member objects.
     */
    ~boustrophedon_path ();

    /**
     * @brief Get the current way point of the path.
     * @return The current way point.
     */
    geometry_msgs::Point current_wp ();

    /**
     * @brief Make the next way point the current one and return it.
     * @returns The next way point.
     */
    geometry_msgs::Point next_wp ();

    /**
     * @brief Get the previous way point.
     * @return The previous way point.
     */
    geometry_msgs::Point previous_wp ();

    /**
     * @brief Make the next way point the current one.
     */
    void step ();

private:
    /**
     * @brief Determine directions parallel and perpendicular to closest bound.
     */
    void find_directions ();

    /**
     * @brief Determine spacing between way points according to camera FOV.
     */
    void find_spacing ();

    /**
     * @brief Determine first way point.
     */
    void find_start ();

    /**
     * @brief Generate all way points for the path.
     */
    void generate_path ();

    /**
     * @brief Get the way point corresponding to a given index.
     * @param idx The index of the way point.
     * @return The way point, if a valid index was given. An empty point otherwise.
     */
    geometry_msgs::Point get_wp (int idx);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief The boustrophedon path.
     */
    deque<geometry_msgs::Point> path;

    /**
     * @brief The bounding area polygon which has to be covered.
     */
    vector<geometry_msgs::Point> area;

    /**
     * @brief The spacing between neighboring path elements.
     */
    double path_spacing;

    /**
     * @brief The direction parallel to the closest bound going right.
     */
    double dir_par_right;

    /**
     * @brief The direction parallel to the closest bound going left.
     */
    double dir_par_left;

    /**
     * @brief The direction perpendicular to the closest bound moving away from the start.
     */
    double dir_perp_out;

    /**
     * @brief The direction perpendicular to the closest bound moving towards the start.
     */
    double dir_perp_in;

    /**
     * @brief Number of way points on paths parallel to closest bound.
     */
    int wp_par;

    /**
     * @brief Number of way points on paths perpendicular to closest bound.
     */
    int wp_perp;

    /**
     * @brief Spacing of way points on paths parallel to closest bound.
     */
    double wp_spacing_par;

    /**
     * @brief Spacing of way points on paths perpendicular to closest bound.
     */
    double wp_spacing_perp;

    /**
     * @brief The indexes of the area bound closest to the CPS's starting position.
     */
    pair<unsigned int, unsigned int> closest_bound;

    /**
     * @brief The CPS's starting position.
     */
    geometry_msgs::Point start;

    /**
     * @brief The number of turns of the boustrophedon path.
     */
    int turns;

    /**
     * @brief The camera field of view (FOV).
     */
    double fov;

    /**
     * @brief The current way point.
     */
    int wp;
};

#endif // BOUSTROPHEDON_PATH_H
