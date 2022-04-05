/*
 * Copyright 2019,
 * Joseph Mirabel
 *
 * LAAS-CNRS
 *
 */

#ifndef __SOT_FEATURE_POSE_HH__
#define __SOT_FEATURE_POSE_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/config.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/matrix-geometry.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/// Enum used to specify what difference operation is used in FeaturePose.
enum Representation_t { SE3Representation, R3xSO3Representation };

/*!
  \brief Feature that controls the relative (or absolute) pose between
  two frames A (or world) and B.

  @tparam representation specify the difference operation to use. This changes
          - the descent direction,
          - the meaning of the mask.
          With R3xSO3Representation, the mask is relative to <em>Frame A</em>.
          If this feature is alone in a SOT, the relative motion of <em>Frame
  B</em> wrt <em>Frame A</em> will be a line. This is what most people want.

  Notations:
  - The frames are refered to with \c fa and \c fb.
  - Each frame is attached to a joint, which are refered to with \c ja and \c
  jb.
  - the difference operator is defined differently depending on the
  representation:
    - R3xSO3Representation:
        \f[ \begin{array}{ccccc}
        \ominus & : & (R^3\times SO(3))^2                 & \to & R^3 \times
  \mathfrak{so}(3) \\
                &   & (a = (t_a,R_a),b = (t_b,R_b))       & \mapsto & b \ominus
  a = (t_b - t_a, \log(R_a^{-1} R_b) \\ \end{array} \f]
    - SE3Representation:
        \f[ \begin{array}{ccccc}
        \ominus & : & SE(3)^2 & \to & \mathfrak{se}(3) \\
                &   & a, b    & \mapsto & b \ominus a = \log(a^{-1} b) \\
        \end{array} \f]

*/
template <Representation_t representation = R3xSO3Representation>
class SOT_CORE_DLLAPI FeaturePose : public FeatureAbstract {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

 public:
  /*! \name Input signals
    @{
  */
  /// Input pose of <em>Joint A</em> wrt to world frame.
  dynamicgraph::SignalPtr<MatrixHomogeneous, int> oMja;
  /// Input pose of <em>Frame A</em> wrt to <em>Joint A</em>.
  dynamicgraph::SignalPtr<MatrixHomogeneous, int> jaMfa;
  /// Input pose of <em>Joint B</em> wrt to world frame.
  dynamicgraph::SignalPtr<MatrixHomogeneous, int> oMjb;
  /// Input pose of <em>Frame B</em> wrt to <em>Joint B</em>.
  dynamicgraph::SignalPtr<MatrixHomogeneous, int> jbMfb;
  /// Jacobian of the input <em>Joint A</em>, expressed in <em>Joint A</em>
  dynamicgraph::SignalPtr<Matrix, int> jaJja;
  /// Jacobian of the input <em>Joint B</em>, expressed in <em>Joint B</em>
  dynamicgraph::SignalPtr<Matrix, int> jbJjb;

  /// The desired pose of <em>Frame B</em> wrt to <em>Frame A</em>.
  dynamicgraph::SignalPtr<MatrixHomogeneous, int> faMfbDes;
  /// The desired velocity of <em>Frame B</em> wrt to <em>Frame A</em>. The
  /// value is expressed in <em>Frame A</em>.
  dynamicgraph::SignalPtr<Vector, int> faNufafbDes;
  /*! @} */

  /*! \name Output signals
    @{
  */
  /// Pose of <em>Frame B</em> wrt to <em>Frame A</em>.
  SignalTimeDependent<MatrixHomogeneous, int> faMfb;

  /// Pose of <em>Frame B</em> wrt to <em>Frame A</em>.
  /// It is expressed as a translation followed by a quaternion.
  SignalTimeDependent<Vector7, int> q_faMfb;

  /// Desired pose of <em>Frame B</em> wrt to <em>Frame A</em>.
  /// It is expressed as a translation followed by a quaternion.
  SignalTimeDependent<Vector7, int> q_faMfbDes;
  /*! @} */

  using FeatureAbstract::errorSOUT;
  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::selectionSIN;

  /*! \name Dealing with the reference value to be reach with this feature.
    @{  */
  DECLARE_NO_REFERENCE();
  /*! @} */

 public:
  FeaturePose(const std::string &name);
  virtual ~FeaturePose(void);

  virtual unsigned int &getDimension(unsigned int &dim, int time);

  /// Computes \f$ {}^oM^{-1}_{fa} {}^oM_{fb} \ominus {}^{fa}M^*_{fb} \f$
  virtual dynamicgraph::Vector &computeError(dynamicgraph::Vector &res,
                                             int time);
  /// Computes \f$ \frac{\partial\ominus}{\partial b} X {}^{fa}\nu^*_{fafb} \f$.
  /// There are two different cases, depending on the representation:
  /// - R3xSO3Representation: \f$ X = \left( \begin{array}{cc} I_3 & [
  /// {}^{fa}t_{fb} ] \\ 0_3 & {{}^{fa}R^*_{fb}}^T \end{array} \right) \f$
  /// - SE3Representation: \f$ X = {{}^{fa}X^*_{fb}}^{-1} \f$ (see
  /// pinocchio::SE3Base<Scalar,Options>::toActionMatrix)
  virtual dynamicgraph::Vector &computeErrorDot(dynamicgraph::Vector &res,
                                                int time);
  /// Computes \f$ \frac{\partial\ominus}{\partial b} Y \left( {{}^{fb}X_{jb}}
  /// {}^{jb}J_{jb} - {{}^{fb}X_{ja}} {}^{ja}J_{ja} \right) \f$. There are two
  /// different cases, depending on the representation:
  /// - R3xSO3Representation: \f$ Y = \left( \begin{array}{cc} {{}^{fa}R_{fb}} &
  /// 0_3 \\ 0_3 & I_3 \end{array} \right) \f$
  /// - SE3Representation: \f$ Y = I_6 \f$
  virtual dynamicgraph::Matrix &computeJacobian(dynamicgraph::Matrix &res,
                                                int time);

  virtual void display(std::ostream &os) const;

 public:
  void servoCurrentPosition(const int &time);

 private:
  MatrixHomogeneous &computefaMfb(MatrixHomogeneous &res, int time);
  Vector7 &computeQfaMfb(Vector7 &res, int time);
  Vector7 &computeQfaMfbDes(Vector7 &res, int time);

  /// \todo Intermediate variables for internal computations
};

template <typename T>
Vector6d convertVelocity(const MatrixHomogeneous &M,
                         const MatrixHomogeneous &Mdes,
                         const Vector &faNufafbDes);

template <>
const std::string FeaturePose<SE3Representation>::CLASS_NAME;
template <>
const std::string FeaturePose<R3xSO3Representation>::CLASS_NAME;

#if __cplusplus >= 201103L
extern template class FeaturePose<SE3Representation>;
extern template class FeaturePose<R3xSO3Representation>;
#endif

typedef FeaturePose<R3xSO3Representation> FeaturePose_t;
typedef FeaturePose<SE3Representation> FeaturePoseSE3_t;

} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_FEATURE_POSE_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
