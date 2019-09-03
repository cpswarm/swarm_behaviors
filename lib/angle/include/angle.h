#ifndef ANGLE_H
#define ANGLE_H

#include <math.h>

/**
 * @brief An angle class.
 */
class angle
{
public:
    /**
     * @brief Constructor.
     */
    angle ();

    /**
     * @brief Constructor.
     * @param a The angle value in radian.
     * @param radian Whether the angle is given as radian, default true.
     */
    angle (double a, bool radian = true);

    /**
     * @brief Set the angle value.
     * @param a The angle value in radian.
     */
    void set (double a);

    /**
     * @brief Set the angle value.
     * @param a The angle value in degree.
     */
    void set_deg (double a);

    /**
     * @brief Get the angle in degree.
     * @return The angle in the range [-180,+180].
     */
    double deg () const;

    /**
     * @brief Get the angle in radian.
     * @return The angle in the range [-π,+π].
     */
    double rad () const;

    /**
     * @brief Get the angle in radian.
     * @return The angle in the range [0,2π].
     */
    double rad_pos () const;

    /**
     * @brief Add an integer value to this angle.
     * @param add The value to add.
     * @return The sum of this angle and the integer value.
     */
    angle operator+ (const int add) const;

    /**
     * @brief Add a double value to this angle.
     * @param add The value to add.
     * @return The sum of this angle and the double value.
     */
    angle operator+ (const double add) const;

    /**
     * @brief Add an angle to this angle.
     * @param add The angle to add.
     * @return The sum of this angle and the angle to add.
     */
    angle operator+ (const angle& add) const;

    /**
     * @brief Add an angle to this angle.
     * @param add The angle to add.
     */
    void operator+= (const angle& add);

    /**
     * @brief Subtract an angle from this angle.
     * @param sub The value to subtract.
     * @return The difference of this angle and the angle to subtract.
     */
    angle operator- (const angle& sub) const;

    /**
     * @brief Subtract an angle from this angle.
     * @param sub The value to subtract.
     */
    void operator-= (const angle& sub);

    /**
     * @brief Multiply an integer value with this angle.
     * @param mult The value to multiply by.
     * @return The product of this angle and the integer value.
     */
    angle operator* (const int mult) const;

    /**
     * @brief Multiply a double value with this angle.
     * @param mult The value to multiply by.
     * @return The product of this angle and the double value.
     */
    angle operator* (const double mult) const;

    /**
     * @brief Divide this angle by an integer value.
     * @param div The value to divide by.
     * @return The fraction of this angle and the integer value.
     */
    angle operator/ (const int div) const;

    /**
     * @brief Divide this angle by a double value.
     * @param div The value to divide by.
     * @return The fraction of this angle and the double value.
     */
    angle operator/ (const double div) const;

    /**
     * @brief Test an angle for equality.
     * @param comp The angle to compare.
     * @return True if this angle equals the given angle, false otherwise.
     */
    bool operator== (const angle& comp) const;

    /**
     * @brief Test if this angle is less than a given angle.
     * @param comp The angle to compare.
     * @return True if this angle is less than the given angle, false otherwise.
     */
    bool operator< (const angle& comp) const;

    /**
     * @brief Test if this angle is greater than a given angle.
     * @param comp The angle to compare.
     * @return True if this angle is greater than the given angle, false otherwise.
     */
    bool operator> (const angle& comp) const;

    /**
     * @brief Test if this angle is less or equal than a given angle.
     * @param comp The angle to compare.
     * @return True if this angle is less or equal to the given angle, false otherwise.
     */
    bool operator<= (const angle& comp) const;

    /**
     * @brief Test if this angle is greater or equal than a given angle.
     * @param comp The angle to compare.
     * @return True if this angle is greater or equal to the given angle, false otherwise.
     */
    bool operator>= (const angle& comp) const;


private:
    /**
     * @brief Normalize the angle in the range [-π,π].
     */
    void normalize ();

    /**
     * @brief Convert the angle from degree to radian.
     * @param a The angle in degree to convert.
     * @return The converted angle in radian.
     */
    double deg_to_rad (double a) const;

    /**
     * @brief Convert the angle from radian to degree.
     * @param a The angle in radian to convert.
     * @return The converted angle in degree.
     */
    double rad_to_deg (double a) const;

    /**
     * @brief The angle value in radian.
     */
    double a;
};

#endif // ANGLE_H
