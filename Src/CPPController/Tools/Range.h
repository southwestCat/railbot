/**
 * @file Range.h
 *
 * The file defines a template class to represent ranges.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include <algorithm>
#include <type_traits>

template <typename T>
class Range
{
public:
    constexpr Range() : min(T()), max(T()){};
    constexpr Range(T minmax) : min(minmax), max(minmax){};

    constexpr Range(T min, T max) : min(min), max(max){};

    static constexpr Range<T> ZeroOneRange();
    static constexpr Range<T> OneRange();

    Range<T> &add(T t)
    {
        if (min > t)
            min = t;
        if (max < t)
            max = t;
        return *this;
    }

    Range<T> &add(const Range<T> &r)
    {
        add(r.min);
        add(r.max);
        return *this;
    }

    constexpr bool isInside(T t) const
    {
        return min <= max ? t >= min && t <= max : t >= min || t <= max;
    }

    constexpr T limit(T t) const
    {
        return t < min ? min : t > max ? max : t;
    }

    constexpr T clamped(T t) const
    {
        return limit(t);
    }

    T &clamp(T &t) const
    {
        t = clamped(t);
        return t;
    }

    template <typename Derived>
    Derived &clamp(Eigen::DenseBase<Derived> &mat) const
    {
        static_assert(std::is_same<typename Eigen::MatrixBase<Derived>::Scalar, T>::value, "Matrix must have the same scalar type as the Range.");
        return mat = mat.derived().unaryExpr([this](T val) { return clamped(val); });
    }

    template <typename Derived>
    Derived clamped(const Eigen::DenseBase<Derived> &mat) const
    {
        static_assert(std::is_same<typename Eigen::MatrixBase<Derived>::Scalar, T>::value, "Matrix must have the same scalar type as the Range.");
        return mat.derived().unaryExpr([this](T val) { return clamped(val); });
    }

    constexpr Range<T> limit(const Range<T>& r) const { return Range<T>(limit(r.min), limit(r.max)); } //sets the limit of a Range

    T scale(T t, const Range<T>& tRange) const;

    constexpr T getSize() const {return max - min;}

    constexpr T getCenter() const {return (max + min) / 2;}

    constexpr bool operator==(const Range<T> &r) const { return min == r.min && max == r.max; }
    constexpr bool operator<(const Range<T> &r) const { return max < r.min; }
    constexpr bool operator>(const Range<T> &r) const { return min > r.max; }
    constexpr bool meets(const Range<T> &r) const { return max == r.min; }
    constexpr bool metBy(const Range<T> &r) const { return min == r.max; }
    constexpr bool overlaps(const Range<T> &r) const { return min < r.min && max < r.max && max > r.min; }
    constexpr bool overlappedBy(const Range<T> &r) const { return min > r.min && max > r.max && min < r.max; }
    constexpr bool starts(const Range<T> &r) const { return min == r.min && max < r.max; }
    constexpr bool startedBy(const Range<T> &r) const { return min == r.min && max > r.max; }
    constexpr bool finishes(const Range<T> &r) const { return max == r.max && min > r.min; }
    constexpr bool finishedBy(const Range<T> &r) const { return max == r.max && min < r.min; }
    constexpr bool during(const Range<T> &r) const { return min > r.min && max < r.max; }
    constexpr bool contains(const Range<T> &r) const { return min < r.min && max > r.max; }

    constexpr bool operator!=(const Range<T>& r) const {return min != r.min || max != r.max;}

    constexpr T intersectionSizeWith(const Range<T>& r) const {return std::max(0.f, std::min(max, r.max) - std::max(min, r.min));}

    T min;
    T max;
};

using Rangea = Range<Angle>;
using Rangei = Range<int>;
using Rangef = Range<float>;
using Rangeuc = Range<unsigned char>;

template<typename T>
constexpr Range<T> Range<T>::ZeroOneRange()
{
  return Range<T>(T(0), T(1));
}

template<typename T>
constexpr Range<T> Range<T>::OneRange()
{
  return Range<T>(T(-1), T(1));
}

template<typename T>
T Range<T>::scale(T t, const Range<T>& tRange) const
{
  return limit(((t - tRange.min) / (tRange.max - tRange.min)) * (max - min) + min);
}
