#include "angle.h"

angle::angle () : angle(0.0)
{
}

angle::angle (double a, bool radian)
{
    if (radian)
        this->a = a;
    else
        this->a = deg_to_rad(a);
    normalize();
}

void angle::set (double a)
{
    this->a = a;
    normalize();
}

void angle::set_deg (double a)
{
    this->a = deg_to_rad(a);
    normalize();
}

double angle::deg () const
{
    if (a > M_PI)
        return rad_to_deg(a - 2 * M_PI);
    else
        rad_to_deg(a);
}

double angle::rad () const
{
    if (a > M_PI)
        return a - 2 * M_PI;
    else
        return a;
}

double angle::rad_pos () const
{
    return a;
}

void angle::normalize ()
{
    while (a > 2 * M_PI) {
        a -= 2 * M_PI;
    }
    while (a < 0) {
        a += 2 * M_PI;
    }
}

double angle::deg_to_rad (double a) const
{
    return a / 180 * M_PI;
}

double angle::rad_to_deg (double a) const
{
    return a / M_PI * 180;
}

angle angle::operator+ (const int add) const
{
    return angle(a + double(add));
}

angle angle::operator+ (const double add) const
{
    return angle(a + add);
}

angle angle::operator+ (const angle& add) const
{
    return angle(a + add.a);
}

void angle::operator+= (const angle& add)
{
    a += add.a;
    normalize();
}

angle angle::operator- (const angle& sub) const
{
    return angle(a - sub.a);
}

void angle::operator-= (const angle& sub)
{
    a -= sub.a;
    normalize();
}

angle angle::operator* (const int mult) const
{
    return angle(a * double(mult));
}

angle angle::operator* (const double mult) const
{
    return angle(a * mult);
}

angle angle::operator/ (const int div) const
{
    return angle(a / double(div));
}

angle angle::operator/ (const double div) const
{
    return angle(a / div);
}

bool angle::operator== (const angle& comp) const
{
    return a == comp.a;
}

bool angle::operator< (const angle& comp) const
{
    return a < comp.a;
}

bool angle::operator> (const angle& comp) const
{
    return a > comp.a;
}

bool angle::operator<= (const angle& comp) const
{
    return a <= comp.a;
}

bool angle::operator>= (const angle& comp) const
{
    return a >= comp.a;
}
