#pragma once

#include <mesh_mppi/types/Vector.hpp>

namespace mesh_mppi
{

// Represents a line segment
class Line
{
public:
    Line(const Vector& a, const Vector& b)
    : a_(a)
    , b_(b)
    , direction_((b - a).normalized())
    , length_((b - a).length())
    {}

    const Vector& a() const {return a_;}
    const Vector& b() const {return b_;}

    inline Vector closest_point_to(const Vector& point) const
    {
        const Vector ap = point - a();
        const float t = std::clamp(direction_.dot(ap), 0.0f, length_);
        return a() + direction_ * t;
    }

private:
    Vector a_;
    Vector b_;

    Vector direction_;
    float length_;
};

// Represents a curve made up of line segments
class PolyLine: public std::vector<Line>
{
public:
    Vector closest_point_to(const Vector& point) const
    {
        float min_dist = std::numeric_limits<float>::infinity();
        Vector closest;
        for (const auto& line: *this)
        {
            const Vector cp = line.closest_point_to(point);
            const float sqd = cp.distance2(point);

            if (sqd < min_dist)
            {
                min_dist = sqd;
                closest = cp;
            }
        }

        return closest;
    }

    const_iterator closest_segment_to(const Vector& point) const
    {
        float min_dist = std::numeric_limits<float>::infinity();
        const_iterator closest;
        for (auto line = begin(); line != end(); ++line)
        {
            const Vector cp = line->closest_point_to(point);
            const float sqd = cp.distance2(point);

            if (sqd < min_dist)
            {
                min_dist = sqd;
                closest = line;
            }
        }

        return closest;
    }

};

} // namespace mesh_mppi
