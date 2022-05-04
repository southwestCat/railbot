#include "HeadLimits.h"
#include "Representations/Infrastructure/JointAngles.h"

#include <algorithm>

Rangea HeadLimits::getTiltBound(Angle pan) const
{
    auto begin = intervals.cbegin();
    auto end = intervals.cend();
    auto it = std::lower_bound(begin, end, pan);

    if (it == end)
        return Rangea(JointAngles::off, JointAngles::off); // Unreachable pan angle

    const size_t index = it - begin;
    const Angle xe = intervals[index];   // Interval end
    const Angle le = lowerBounds[index]; // Lower bound at interval end
    const Angle ue = upperBounds[index]; // Upper bound at interval end
    if (pan == xe)
        return Rangea(le, ue);

    if (index == 0)
        return Rangea(JointAngles::off, JointAngles::off); // Unreachable pan angle (smaller than begin of first interval)

    const Angle xs = intervals[index - 1];   // Interval start
    const Angle ls = lowerBounds[index - 1]; // Lower bound at interval start.
    const Angle us = upperBounds[index - 1]; // Upper bound at interval start

    const Angle lowerSlope = (le - ls) / (xe - xs);
    const Angle upperSlope = (ue - us) / (xe - xs);
    return Rangea(ls + lowerSlope * (pan - xs), us + upperSlope * (pan - xs));
}