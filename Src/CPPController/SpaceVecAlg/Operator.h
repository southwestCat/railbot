#pragma once

#include "Tools/Math/Eigen.h"

namespace sva
{

// Operators implementation

namespace sva_internal
{

template<typename Derived1, typename Derived2, typename Derived3>
inline void colwiseCrossEq(const Eigen::MatrixBase<Derived1> & m1,
                           const Eigen::MatrixBase<Derived2> & m2,
                           Eigen::MatrixBase<Derived3> const & result)
{
  Eigen::MatrixBase<Derived3> & result_nc = const_cast<Eigen::MatrixBase<Derived3> &>(result);
  result_nc.row(0) = (m1.row(1) * m2.coeff(2) - m1.row(2) * m2.coeff(1));
  result_nc.row(1) = (m1.row(2) * m2.coeff(0) - m1.row(0) * m2.coeff(2));
  result_nc.row(2) = (m1.row(0) * m2.coeff(1) - m1.row(1) * m2.coeff(0));
}

template<typename Derived1, typename Derived2, typename Derived3>
inline void colwiseCrossPlusEq(const Eigen::MatrixBase<Derived1> & m1,
                               const Eigen::MatrixBase<Derived2> & m2,
                               Eigen::MatrixBase<Derived3> const & result)
{
  Eigen::MatrixBase<Derived3> & result_nc = const_cast<Eigen::MatrixBase<Derived3> &>(result);
  result_nc.row(0) += (m1.row(1) * m2.coeff(2) - m1.row(2) * m2.coeff(1));
  result_nc.row(1) += (m1.row(2) * m2.coeff(0) - m1.row(0) * m2.coeff(2));
  result_nc.row(2) += (m1.row(0) * m2.coeff(1) - m1.row(1) * m2.coeff(0));
}

template<typename Derived1, typename Derived2, typename Derived3>
inline void colwiseCrossMinusEq(const Eigen::MatrixBase<Derived1> & m1,
                                const Eigen::MatrixBase<Derived2> & m2,
                                Eigen::MatrixBase<Derived3> const & result)
{
  Eigen::MatrixBase<Derived3> & result_nc = const_cast<Eigen::MatrixBase<Derived3> &>(result);
  result_nc.row(0) -= (m1.row(1) * m2.coeff(2) - m1.row(2) * m2.coeff(1));
  result_nc.row(1) -= (m1.row(2) * m2.coeff(0) - m1.row(0) * m2.coeff(2));
  result_nc.row(2) -= (m1.row(0) * m2.coeff(1) - m1.row(1) * m2.coeff(0));
}

template<typename Derived1, typename Derived2, typename Derived3>
inline void colwiseLeftMultEq(const Eigen::MatrixBase<Derived1> & m1,
                              const Eigen::MatrixBase<Derived2> & m2,
                              Eigen::MatrixBase<Derived3> const & result)
{
  Eigen::MatrixBase<Derived3> & result_nc = const_cast<Eigen::MatrixBase<Derived3> &>(result);
  for(typename Derived1::Index i = 0; i < m1.cols(); ++i)
  {
    result_nc.col(i) = m2 * m1.col(i);
  }
}


template<typename Derived>
inline Eigen::Block<Derived, 3, Eigen::Dynamic> motionAngular(Eigen::MatrixBase<Derived> & mv)
{
  return Eigen::Block<Derived, 3, Eigen::Dynamic>(mv.derived(), 0, 0, 3, mv.cols());
}

template<typename Derived>
inline Eigen::Block<const Derived, 3, Eigen::Dynamic> motionAngular(const Eigen::MatrixBase<Derived> & mv)
{
  return Eigen::Block<const Derived, 3, Eigen::Dynamic>(mv.derived(), 0, 0, 3, mv.cols());
}

template<typename Derived>
inline Eigen::Block<Derived, 3, Eigen::Dynamic> motionLinear(Eigen::MatrixBase<Derived> & mv)
{
  return Eigen::Block<Derived, 3, Eigen::Dynamic>(mv.derived(), 3, 0, 3, mv.cols());
}

template<typename Derived>
inline Eigen::Block<const Derived, 3, Eigen::Dynamic> motionLinear(const Eigen::MatrixBase<Derived> & mv)
{
  return Eigen::Block<const Derived, 3, Eigen::Dynamic>(mv.derived(), 3, 0, 3, mv.cols());
}

template<typename Derived>
inline Eigen::Block<Derived, 3, Eigen::Dynamic> forceCouple(Eigen::MatrixBase<Derived> & mv)
{
  return Eigen::Block<Derived, 3, Eigen::Dynamic>(mv.derived(), 0, 0, 3, mv.cols());
}

template<typename Derived>
inline Eigen::Block<const Derived, 3, Eigen::Dynamic> forceCouple(const Eigen::MatrixBase<Derived> & mv)
{
  return Eigen::Block<const Derived, 3, Eigen::Dynamic>(mv.derived(), 0, 0, 3, mv.cols());
}

template<typename Derived>
inline Eigen::Block<Derived, 3, Eigen::Dynamic> forceForce(Eigen::MatrixBase<Derived> & mv)
{
  return Eigen::Block<Derived, 3, Eigen::Dynamic>(mv.derived(), 3, 0, 3, mv.cols());
}

template<typename Derived>
inline Eigen::Block<const Derived, 3, Eigen::Dynamic> forceForce(const Eigen::MatrixBase<Derived> & mv)
{
  return Eigen::Block<const Derived, 3, Eigen::Dynamic>(mv.derived(), 3, 0, 3, mv.cols());
}


} // namespace sva_internal



}