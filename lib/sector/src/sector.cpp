#include "sector.h"

sector::sector (double min, double max)
{
    // check if full circle is given
    if (max == min + 2.0 * M_PI)
        is_full = true;
    else
        is_full = false;

    // initialize bearing
    angle bear_min = angle(min);
    angle bear_max = angle(max);
    bearing.first = std::min(bear_min, bear_max);
    bearing.second = std::max(bear_min, bear_max);

    // sector is the inverse of the bearing
    is_inverse = bear_min > bear_max;
}

sector::sector (angle min, angle max)
{
    // initialize bearing
    bearing.first = std::min(min, max);
    bearing.second = std::max(min, max);

    // sector is the inverse of the bearing
    is_inverse = min > max;
}

sector::sector (const sector& sec)
{
    // copy members
    bearing.first = sec.bearing.first;
    bearing.second = sec.bearing.second;
    is_inverse = sec.is_inverse;
    is_full = sec.is_full;
}

double sector::center () const
{
    angle center;

    if (is_inverse) {
        center = bearing.second + angle(size() / 2);
    }
    else {
        center = bearing.first + angle(size() / 2);
    }

    return center.rad_pos();
}

bool sector::contains (double a) const
{
    angle ang = angle(a);

    return contains(ang);
}

bool sector::contains (angle a) const
{
    // check if bearing is within sector
    if (is_full)
        return true;
    else if (is_inverse) {
        return angle(0) <= a && a <= bearing.first || bearing.second <= a && a <= angle(2 * M_PI);
    }
    else {
        return bearing.first <= a && a <= bearing.second;
    }
}

void sector::inflate ()
{
    bearing.first = angle(0);
    bearing.second = angle(2.0 * M_PI);
    is_inverse = false;
    is_full = true;

    normalize();
}

void sector::inflate (double a)
{
    angle ang = angle(a);

    // already full circle, nothing todo
    if (is_full)
        return;

    // inflate sector to full circle
    if ((ang * 2).rad_pos() + size() >= sector(0, 2 * M_PI).size()) {
        bearing.first = angle(inverse().center());
        bearing.second = bearing.first;
        if (bearing.first == 0)
            is_inverse = false;
        else
            is_inverse = true;
        is_full = true;
    }

    // inflate sector including 0
    else if (is_inverse) {
        bearing.first += ang;
        bearing.second -= ang;
    }

    // inflate regular sector
    else {
        bearing.first -= ang;
        bearing.second += ang;
    }

    // normalize sector bearing
    normalize();
}

sector sector::inverse () const
{
    if (is_full)
        return sector(bearing.first.rad(), bearing.second.rad());
    if (is_inverse) {
        if (empty())
            return sector(bearing.first.rad(), bearing.second.rad() + 2 * M_PI);
        else
            return sector(bearing.first.rad(), bearing.second.rad());
    }
    else {
        if (empty())
            return sector(bearing.second.rad(), bearing.first.rad() + 2 * M_PI);
        else
            return sector(bearing.second.rad(), bearing.first.rad());
    }
}

void sector::join (sector& s)
{
    // this sector is full circle, do nothing
    if (is_full)
        return;

    // s is full circle, take new sector
    if (s.is_full) {
        *this = s;
        return;
    }

    // empty sectors
    if (s.size() == 0)
        return;
    else if (size() == 0) {
        *this = s;
        return;
    }

    // new sector bearings
    double first,second;

    // disjoint sectors, choose smallest possible sector
    if (contains(s.min()) == false && contains(s.max()) == false && s.contains(min()) == false && s.contains(max()) == false) {
        // two possibilities for joining
        sector sector1 = sector(min(), s.max_ord());
        sector sector2 = sector(s.min(), max_ord());

        // first sector is smaller
        if (sector1 < sector2) {
            *this = sector1;
        }

        // second sector is smaller
        else {
            *this = sector2;
        }
    }

    // overlapping sectors, join them
    else {
        // this sector contains both bearins of s
        if (contains(s.min()) && contains(s.max())) {
            // joined sectors cover full circle
            if (s.contains(min()) && s.contains(max())) {
                inflate();
            }

            // else, this sector contains s, nothing to do
        }

        // this sector contains minimum bearing of s
        else if (contains(s.min())) {
            if (is_inverse && s.is_inverse){
                first = std::max(max(), s.max());
                second = std::min(min(), s.min());
            }
            else if (is_inverse) {
                first = std::max(max(), s.max());
                second = std::max(min(), s.min());
            }
            else if (s.is_inverse) {
                first = std::min(min(), s.min());
                second = std::min(max(), s.max());
            }
            else {
                first = std::min(min(), s.min());
                second = std::max(max(), s.max());
            }

            // assign new bearings
            bearing.first = first;
            bearing.second = second;
        }

        // this sector contains maximum bearing of s
        else if (contains(s.max())) {
            if (is_inverse && s.is_inverse){
                first = std::max(max(), s.max());
                second = std::min(min(), s.min());
            }
            else if (is_inverse) {
                first = std::min(max(), s.max());
                second = std::min(min(), s.min());
            }
            else if (s.is_inverse) {
                first = std::max(min(), s.min());
                second = std::max(max(), s.max());
            }
            else {
                first = std::min(min(), s.min());
                second = std::max(max(), s.max());
            }

            // assign new bearings
            bearing.first = first;
            bearing.second = second;
        }

        // sector s contains this sector
        else {
            *this = s;
        }
    }


    normalize();
}

double sector::max () const
{
    if (is_inverse)
        return bearing.first.rad_pos();
    else
        return bearing.second.rad_pos();
}

double sector::max_ord () const
{
    if (is_full || is_inverse)
        return 2 * M_PI + bearing.first.rad_pos();
    else
        return bearing.second.rad_pos();
}

double sector::min () const
{
    if (is_inverse)
        return bearing.second.rad_pos();
    else
        return bearing.first.rad_pos();
}

double sector::min_ord () const
{
    return min();
}

double sector::size () const
{
    if (is_full)
        return 2.0 * M_PI;
    else if (is_inverse)
        return 2.0 * M_PI - (bearing.second - bearing.first).rad_pos();
    else
        return (bearing.second - bearing.first).rad_pos();
}

bool sector::empty () const
{
    return bearing.first == bearing.second && is_full == false;
}

void sector::normalize ()
{
    // switch min and max
    if (bearing.first > bearing.second) {
        is_inverse = !is_inverse;
        angle temp = bearing.first;
        bearing.first = bearing.second;
        bearing.second = temp;
    }
}

void sector::operator= (const sector& sec)
{
    // copy members
    bearing.first = sec.bearing.first;
    bearing.second = sec.bearing.second;
    is_inverse = sec.is_inverse;
    is_full = sec.is_full;
}

bool sector::operator< (const sector& comp) const
{
    return size() < comp.size();
}
