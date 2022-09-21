/*
 * Copyright 2019
 * Joseph Mirabel
 *
 * LAAS-CNRS
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- SOT --- */
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>

#include <Eigen/LU>
#include <boost/mpl/if.hpp>
#include <boost/type_traits/is_same.hpp>
#include <pinocchio/multibody/liegroup/liegroup.hpp>
#include <sot/core/debug.hh>
#include <sot/core/factory.hh>
#include <sot/core/feature-pose.hh>

namespace dynamicgraph {
namespace sot {

typedef pinocchio::CartesianProductOperation<
    pinocchio::VectorSpaceOperationTpl<3, double>,
    pinocchio::SpecialOrthogonalOperationTpl<3, double> >
    R3xSO3_t;
typedef pinocchio::SpecialEuclideanOperationTpl<3, double> SE3_t;

namespace internal {
template <Representation_t representation>
struct LG_t {
  typedef typename boost::mpl::if_c<representation == SE3Representation, SE3_t,
                                    R3xSO3_t>::type type;
};
}  // namespace internal

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

static const MatrixHomogeneous Id(MatrixHomogeneous::Identity());

template <Representation_t representation>
FeaturePose<representation>::FeaturePose(const std::string &pointName)
    : FeatureAbstract(pointName),
      oMja(NULL, CLASS_NAME + "(" + name + ")::input(matrixHomo)::oMja"),
      jaMfa(NULL, CLASS_NAME + "(" + name + ")::input(matrixHomo)::jaMfa"),
      oMjb(NULL, CLASS_NAME + "(" + name + ")::input(matrixHomo)::oMjb"),
      jbMfb(NULL, CLASS_NAME + "(" + name + ")::input(matrixHomo)::jbMfb"),
      jaJja(NULL, CLASS_NAME + "(" + name + ")::input(matrix)::jaJja"),
      jbJjb(NULL, CLASS_NAME + "(" + name + ")::input(matrix)::jbJjb"),
      faMfbDes(NULL,
               CLASS_NAME + "(" + name + ")::input(matrixHomo)::faMfbDes"),
      faNufafbDes(NULL,
                  CLASS_NAME + "(" + name + ")::input(vector)::faNufafbDes"),
      faMfb(
          boost::bind(&FeaturePose<representation>::computefaMfb, this, _1, _2),
          oMja << jaMfa << oMjb << jbMfb,
          CLASS_NAME + "(" + name + ")::output(matrixHomo)::faMfb"),
      q_faMfb(boost::bind(&FeaturePose<representation>::computeQfaMfb, this, _1,
                          _2),
              faMfb, CLASS_NAME + "(" + name + ")::output(vector7)::q_faMfb"),
      q_faMfbDes(boost::bind(&FeaturePose<representation>::computeQfaMfbDes,
                             this, _1, _2),
                 faMfbDes,
                 CLASS_NAME + "(" + name + ")::output(vector7)::q_faMfbDes") {
  oMja.setConstant(Id);
  jaMfa.setConstant(Id);
  jbMfb.setConstant(Id);
  faMfbDes.setConstant(Id);
  faNufafbDes.setConstant(Vector::Zero(6));

  jacobianSOUT.addDependencies(q_faMfbDes << q_faMfb << jaJja << jbJjb);

  errorSOUT.addDependencies(q_faMfbDes << q_faMfb);

  signalRegistration(oMja << jaMfa << oMjb << jbMfb << jaJja << jbJjb);
  signalRegistration(faMfb << errordotSOUT << faMfbDes << faNufafbDes);

  errordotSOUT.setFunction(
      boost::bind(&FeaturePose<representation>::computeErrorDot, this, _1, _2));
  errordotSOUT.addDependencies(q_faMfbDes << q_faMfb << faNufafbDes);

  // Commands
  //
  {
    using namespace dynamicgraph::command;
    addCommand("keep",
               makeCommandVoid1(
                   *this, &FeaturePose<representation>::servoCurrentPosition,
                   docCommandVoid1(
                       "modify the desired position to servo at current pos.",
                       "time")));
  }
}

template <Representation_t representation>
FeaturePose<representation>::~FeaturePose() {}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

template <Representation_t representation>
static inline void check(const FeaturePose<representation> &ft) {
  (void)ft;
  assert(ft.oMja.isPlugged());
  assert(ft.jaMfa.isPlugged());
  assert(ft.oMjb.isPlugged());
  assert(ft.jbMfb.isPlugged());
  assert(ft.faMfbDes.isPlugged());
  assert(ft.faNufafbDes.isPlugged());
}

template <Representation_t representation>
unsigned int &FeaturePose<representation>::getDimension(unsigned int &dim,
                                                        int time) {
  sotDEBUG(25) << "# In {" << std::endl;

  const Flags &fl = selectionSIN.access(time);

  dim = 0;
  for (int i = 0; i < 6; ++i)
    if (fl(i)) dim++;

  sotDEBUG(25) << "# Out }" << std::endl;
  return dim;
}

void toVector(const MatrixHomogeneous &M, Vector7 &v) {
  v.head<3>() = M.translation();
  QuaternionMap(v.tail<4>().data()) = M.linear();
}

Vector7 toVector(const MatrixHomogeneous &M) {
  Vector7 ret;
  toVector(M, ret);
  return ret;
}

template <Representation_t representation>
Matrix &FeaturePose<representation>::computeJacobian(Matrix &J, int time) {
  typedef typename internal::LG_t<representation>::type LieGroup_t;

  check(*this);

  q_faMfb.recompute(time);
  q_faMfbDes.recompute(time);

  const unsigned int &dim = dimensionSOUT(time);
  const Flags &fl = selectionSIN(time);

  const Matrix &_jbJjb = jbJjb(time);

  const MatrixHomogeneous &_jbMfb =
      (jbMfb.isPlugged() ? jbMfb.accessCopy() : Id);

  const Matrix::Index cJ = _jbJjb.cols();
  J.resize(dim, cJ);

  MatrixTwist X;
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Jminus;

  buildFrom(_jbMfb.inverse(Eigen::Affine), X);
  MatrixRotation faRfb = jaMfa.access(time).rotation().transpose() *
                         oMja.access(time).rotation().transpose() *
                         oMjb.access(time).rotation() * _jbMfb.rotation();
  if (boost::is_same<LieGroup_t, R3xSO3_t>::value)
    X.topRows<3>().applyOnTheLeft(faRfb);
  LieGroup_t().template dDifference<pinocchio::ARG1>(
      q_faMfbDes.accessCopy(), q_faMfb.accessCopy(), Jminus);

  // Contribution of b:
  // J = Jminus * X * jbJjb;
  unsigned int rJ = 0;
  for (unsigned int r = 0; r < 6; ++r)
    if (fl((int)r)) J.row(rJ++) = (Jminus * X).row(r) * _jbJjb;

  if (jaJja.isPlugged()) {
    const Matrix &_jaJja = jaJja(time);
    const MatrixHomogeneous &_jaMfa =
                                (jaMfa.isPlugged() ? jaMfa.accessCopy() : Id),
                            _faMfb = faMfb.accessCopy();

    buildFrom((_jaMfa * _faMfb).inverse(Eigen::Affine), X);
    if (boost::is_same<LieGroup_t, R3xSO3_t>::value)
      X.topRows<3>().applyOnTheLeft(faRfb);

    // J -= (Jminus * X) * jaJja(time);
    rJ = 0;
    for (unsigned int r = 0; r < 6; ++r)
      if (fl((int)r)) J.row(rJ++).noalias() -= (Jminus * X).row(r) * _jaJja;
  }

  return J;
}

template <Representation_t representation>
MatrixHomogeneous &FeaturePose<representation>::computefaMfb(
    MatrixHomogeneous &res, int time) {
  check(*this);

  res = (oMja(time) * jaMfa(time)).inverse(Eigen::Affine) * oMjb(time) *
        jbMfb(time);
  return res;
}

template <Representation_t representation>
Vector7 &FeaturePose<representation>::computeQfaMfb(Vector7 &res, int time) {
  check(*this);

  toVector(faMfb(time), res);
  return res;
}

template <Representation_t representation>
Vector7 &FeaturePose<representation>::computeQfaMfbDes(Vector7 &res, int time) {
  check(*this);

  toVector(faMfbDes(time), res);
  return res;
}

template <Representation_t representation>
Vector &FeaturePose<representation>::computeError(Vector &error, int time) {
  typedef typename internal::LG_t<representation>::type LieGroup_t;
  check(*this);

  const Flags &fl = selectionSIN(time);

  Eigen::Matrix<double, 6, 1> v;
  LieGroup_t().difference(q_faMfbDes(time), q_faMfb(time), v);

  error.resize(dimensionSOUT(time));
  unsigned int cursor = 0;
  for (unsigned int i = 0; i < 6; ++i)
    if (fl((int)i)) error(cursor++) = v(i);

  return error;
}

// This function is responsible of converting the input velocity expressed with
// SE(3) convention onto a velocity expressed with the convention of this
// feature (R^3xSO(3) or SE(3)), in the right frame.
template <>
Vector6d convertVelocity<SE3_t>(const MatrixHomogeneous &M,
                                const MatrixHomogeneous &Mdes,
                                const Vector &faNufafbDes) {
  (void)M;
  MatrixTwist X;
  buildFrom(Mdes.inverse(Eigen::Affine), X);
  return X * faNufafbDes;
}
template <>
Vector6d convertVelocity<R3xSO3_t>(const MatrixHomogeneous &M,
                                   const MatrixHomogeneous &Mdes,
                                   const Vector &faNufafbDes) {
  Vector6d nu;
  nu.head<3>() =
      faNufafbDes.head<3>() - M.translation().cross(faNufafbDes.tail<3>());
  nu.tail<3>() = Mdes.rotation().transpose() * faNufafbDes.tail<3>();
  return nu;
}

template <Representation_t representation>
Vector &FeaturePose<representation>::computeErrorDot(Vector &errordot,
                                                     int time) {
  typedef typename internal::LG_t<representation>::type LieGroup_t;
  check(*this);

  errordot.resize(dimensionSOUT(time));
  const Flags &fl = selectionSIN(time);
  if (!faNufafbDes.isPlugged()) {
    errordot.setZero();
    return errordot;
  }

  q_faMfb.recompute(time);
  q_faMfbDes.recompute(time);
  faNufafbDes.recompute(time);

  const MatrixHomogeneous &_faMfbDes = faMfbDes(time);

  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> Jminus;

  LieGroup_t().template dDifference<pinocchio::ARG0>(
      q_faMfbDes.accessCopy(), q_faMfb.accessCopy(), Jminus);
  Vector6d nu = convertVelocity<LieGroup_t>(faMfb(time), _faMfbDes,
                                            faNufafbDes.accessCopy());
  unsigned int cursor = 0;
  for (unsigned int i = 0; i < 6; ++i)
    if (fl((int)i)) errordot(cursor++) = Jminus.row(i) * nu;

  return errordot;
}

/* Modify the value of the reference (sdes) so that it corresponds
 * to the current position. The effect on the servo is to maintain the
 * current position and correct any drift. */
template <Representation_t representation>
void FeaturePose<representation>::servoCurrentPosition(const int &time) {
  check(*this);

  const MatrixHomogeneous &_oMja = (oMja.isPlugged() ? oMja.access(time) : Id),
                          _jaMfa =
                              (jaMfa.isPlugged() ? jaMfa.access(time) : Id),
                          _oMjb = oMjb.access(time),
                          _jbMfb =
                              (jbMfb.isPlugged() ? jbMfb.access(time) : Id);
  faMfbDes = (_oMja * _jaMfa).inverse(Eigen::Affine) * _oMjb * _jbMfb;
}

static const char *featureNames[] = {"X ", "Y ", "Z ", "RX", "RY", "RZ"};
template <Representation_t representation>
void FeaturePose<representation>::display(std::ostream &os) const {
  os << CLASS_NAME << "<" << name << ">: (";

  try {
    const Flags &fl = selectionSIN.accessCopy();
    bool first = true;
    for (int i = 0; i < 6; ++i)
      if (fl(i)) {
        if (first) {
          first = false;
        } else {
          os << ",";
        }
        os << featureNames[i];
      }
    os << ") ";
  } catch (ExceptionAbstract e) {
    os << " selectSIN not set.";
  }
}

}  // namespace sot
}  // namespace dynamicgraph
