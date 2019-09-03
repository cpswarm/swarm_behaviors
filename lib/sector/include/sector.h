#ifndef SECTOR_H
#define SECTOR_H

#include <utility>
#include <algorithm>
#include "angle.h"

using namespace std;

/**
 * @brief A sector defined by a minimum and maximum bearing.
 */
class sector
{
public:
    /**
     * @brief Constructor that initializes the private member variables. If min and max are equal, the sector is empty. If max is min + 2π, the sector contains the full circle.
     * @param min Minimum bearing of the sector.
     * @param max Maximum bearing of the sector.
     */
    sector (double min, double max);

    /**
     * @brief Constructor that initializes the private member variables from two angles.
     * @param min Minimum bearing of the sector.
     * @param max Maximum bearing of the sector.
     */
    sector (angle min, angle max);

    /**
     * @brief Copy constructor.
     * @param sec The sector to copy.
     */
    sector (const sector& sec);

    /**
     * @brief Get the bearing of the center of the sector.
     * @return The center bearing.
     */
    double center () const;

    /**
     * @brief Check whether the sector contains the given angle.
     * @param a The angle to check.
     * @return True, if the angle lies within the sector, false otherwise.
     */
    bool contains (double a) const;

    /**
     * @brief Check whether the sector contains the given angle.
     * @param a The angle to check.
     * @return True, if the angle lies within the sector, false otherwise.
     */
    bool contains (angle a) const;

    /**
     * @brief Increase the sector to a full circle.
     */
    void inflate ();

    /**
     * @brief Increase the sector by adding the given angle on both sides.
     * @param a The angle to add on both sides.
     */
    void inflate (double a);

    /**
     * @brief Get the inverse sector which covers the rest of a circle.
     * @return The inverse sector.
     */
    sector inverse () const;

    /**
     * @brief Join two sectors. The resulting sector includes both sectors. If the sectors are disjoint, the resulting sector also includes the smaller angle enclosed by the two sectors.
     * @param s The second sector to join.
     */
    void join (sector& s);

    /**
     * @brief Get the maximum bearing of the sector.
     * @return The maximum bearing in [0,2π].
     */
    double max () const;

    /**
     * @brief Get the maximum bearing of the sector such that it is greater than the minimum bearing.
     * @return The maximum bearing in (0,4π).
     */
    double max_ord () const;

    /**
     * @brief Get the minimum bearing of the sector.
     * @return The minimum bearing in [0,2π].
     */
    double min () const;

    /**
     * @brief Get the minimum bearing of the sector such that it is smaller than the maximum bearing.
     * @return The minimum bearing in [0,2π].
     */
    double min_ord () const;

    /**
     * @brief Get the size in radian covered by the sector.
     * @return The size of this sector.
     */
    double size () const;

private:
    /**
     * @brief Check wheter the sector is empty.
     * @return True, if maximum and minimum bearing are the same, false otherwise.j
     */
    bool empty () const;

    /**
     * @brief Normalize minimum and maximum bearing.
     */
    void normalize ();

    /**
     * @brief Assignment operator.
     * @param sec The sector to assign to this sector.
     */
    void operator= (const sector& sec);

    /**
     * @brief Compare the size of two sectors.
     * @param comp The sector to compare.
     */
    bool operator< (const sector& comp) const;

    /**
     * @brief The minimum and maximum bearing of the sector. The first value always represents the lower bearing.
     */
    pair<angle,angle> bearing;

    /**
     * @brief Whether the sector includes the jump from 2π to 0.
     */
    bool is_inverse;

    /**
     * @brief Whether the sector includes the full circle.
     */
    bool is_full;
};

#endif // SECTOR_H
