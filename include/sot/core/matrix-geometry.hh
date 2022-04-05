/*
 * Copyright 2010,2019
 *  CNRS/AIST
 * François Bleibel, Olivier Stasse, François Bailly
 *
 */

#ifndef __SOT_MATRIX_GEOMETRY_H__
#define __SOT_MATRIX_GEOMETRY_H__

/* --- Matrix --- */
#include <dynamic-graph/eigen-io.h>
#include <dynamic-graph/linear-algebra.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sot/core/api.hh>

#define MRAWDATA(x) x.data()

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)           \
  /** \ingroup matrixtypedefs */                                          \
  typedef Eigen::Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix; \
  /** \ingroup matrixtypedefs */                                          \
  typedef Eigen::Matrix<Type, Size, 1> Vector##SizeSuffix##TypeSuffix;    \
  /** \ingroup matrixtypedefs */                                          \
  typedef Eigen::Matrix<Type, 1, Size> RowVector##SizeSuffix##TypeSuffix;

#define EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, Size) \
  /** \ingroup matrixtypedefs */                          \
  typedef Eigen::Matrix<Type, Size, Eigen::Dynamic>       \
      Matrix##Size##X##TypeSuffix;                        \
  /** \ingroup matrixtypedefs */                          \
  typedef Eigen::Matrix<Type, Eigen::Dynamic, Size> Matrix##X##Size##TypeSuffix;

#define EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix) \
  EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 1, 1)           \
  EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 5, 5)           \
  EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 6, 6)           \
  EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 7, 7)           \
  EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 1)        \
  EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 5)        \
  EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 6)        \
  EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 7)

EIGEN_MAKE_TYPEDEFS_ALL_SIZES(int, i)
EIGEN_MAKE_TYPEDEFS_ALL_SIZES(float, f)
EIGEN_MAKE_TYPEDEFS_ALL_SIZES(double, d)
EIGEN_MAKE_TYPEDEFS_ALL_SIZES(std::complex<float>, cf)
EIGEN_MAKE_TYPEDEFS_ALL_SIZES(std::complex<double>, cd)

#undef EIGEN_MAKE_TYPEDEFS_ALL_SIZES
#undef EIGEN_MAKE_TYPEDEFS

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    MatrixRXd;
typedef Eigen::Map<MatrixRXd> SigMatrixXd;
typedef Eigen::Map<Eigen::VectorXd> SigVectorXd;
typedef const Eigen::Map<const MatrixRXd> const_SigMatrixXd;
typedef const Eigen::Map<const Eigen::VectorXd> const_SigVectorXd;

typedef Eigen::Ref<Eigen::VectorXd> RefVector;
typedef const Eigen::Ref<const Eigen::VectorXd> &ConstRefVector;
typedef Eigen::Ref<Eigen::MatrixXd> RefMatrix;
typedef const Eigen::Ref<const Eigen::MatrixXd> ConstRefMatrix;

typedef Eigen::Transform<double, 3, Eigen::Affine> SOT_CORE_EXPORT
    MatrixHomogeneous;
typedef Eigen::Matrix<double, 3, 3> SOT_CORE_EXPORT MatrixRotation;
typedef Eigen::AngleAxis<double> SOT_CORE_EXPORT VectorUTheta;
typedef Eigen::Quaternion<double> SOT_CORE_EXPORT VectorQuaternion;
typedef Eigen::Vector3d SOT_CORE_EXPORT VectorRotation;
typedef Eigen::Vector3d SOT_CORE_EXPORT VectorRollPitchYaw;
typedef Eigen::Matrix<double, 6, 6> SOT_CORE_EXPORT MatrixForce;
typedef Eigen::Matrix<double, 6, 6> SOT_CORE_EXPORT MatrixTwist;

typedef Eigen::Matrix<double, 7, 1> SOT_CORE_EXPORT Vector7;
typedef Eigen::Quaternion<double> SOT_CORE_EXPORT Quaternion;
typedef Eigen::Map<Quaternion> SOT_CORE_EXPORT QuaternionMap;

inline void buildFrom(const MatrixHomogeneous &MH, MatrixTwist &MT) {
  Eigen::Vector3d _t = MH.translation();
  MatrixRotation R(MH.linear());
  Eigen::Matrix3d Tx;
  Tx << 0, -_t(2), _t(1), _t(2), 0, -_t(0), -_t(1), _t(0), 0;
  Eigen::Matrix3d sk;
  sk = Tx * R;

  MT.block<3, 3>(0, 0) = R;
  MT.block<3, 3>(0, 3) = sk;
  MT.block<3, 3>(3, 0).setZero();
  MT.block<3, 3>(3, 3) = R;
}

}  // namespace sot
}  // namespace dynamicgraph

#endif /* #ifndef __SOT_MATRIX_GEOMETRY_H__ */
