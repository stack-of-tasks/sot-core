/*
 * Copyright 2019,
 * Joseph Mirabel
 *
 * LAAS-CNRS
 *
 */

#ifndef __SOT_FEATURE_TRANSFORMATION_HH__
#define __SOT_FEATURE_TRANSFORMATION_HH__

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

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \brief Feature that controls the relative (or absolute) pose between
  two frames A (or world) and B.

  Notations:
  \li The frames are refered to with \c fa and \c fb.
  \li Each frame is attached to a joint, which are refered to with \c ja and \c jb.
  \li the difference operator is defined as \f[
  \begin{array}{ccccc}
  \ominus & : & SE(3)^2 & \to & \mathfrak{se}(3) \\
          &   & a, b    & \mapsto & b \ominus a = \log(a^{-1} b) \\
   \end{array}
  \f]
  \todo express error in R3xS03 so that the mask isn't surprising for user.
*/
class SOT_CORE_DLLAPI FeaturePose
  : public FeatureAbstract
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  /*! \name Input signals
    @{
  */
  /// Input pose of <em>Joint A</em> wrt to world frame.
  dg::SignalPtr< MatrixHomogeneous, int > oMja;
  /// Input pose of <em>Frame A</em> wrt to <em>Joint A</em>.
  dg::SignalPtr< MatrixHomogeneous, int > jaMfa;
  /// Input pose of <em>Joint B</em> wrt to world frame.
  dg::SignalPtr< MatrixHomogeneous, int > oMjb;
  /// Input pose of <em>Frame B</em> wrt to <em>Joint B</em>.
  dg::SignalPtr< MatrixHomogeneous, int > jbMfb;
  /// Jacobian of the input <em>Joint A</em>, expressed in <em>Joint A</em>
  dg::SignalPtr< Matrix,int > jaJja;
  /// Jacobian of the input <em>Joint B</em>, expressed in <em>Joint B</em>
  dg::SignalPtr< Matrix,int > jbJjb;

  /// The desired pose of <em>Frame B</em> wrt to <em>Frame A</em>.
  dg::SignalPtr< MatrixHomogeneous, int > faMfbDes;
  /// The desired velocity of <em>Frame B</em> wrt to <em>Frame A</em>. The value is expressed in <em>Frame A</em>.
  dg::SignalPtr< Vector, int > faNufafb;
  /*! @} */

  /*! \name Output signals
    @{
  */
  /// Pose of <em>Frame B</em> wrt to <em>world frame</em>.
  /// It is expressed as a translation followed by a quaternion.
  SignalTimeDependent< Vector7, int > q_oMfb;

  /// Pose of <em>Frame B*</em> wrt to <em>world frame</em>.
  /// It is expressed as a translation followed by a quaternion.
  SignalTimeDependent< Vector7, int > q_oMfbDes;
  /*! @} */

  using FeatureAbstract::selectionSIN;
  // TODO Rename into dError_dq or Jerror
  using FeatureAbstract::jacobianSOUT;
  // TODO Rename into error
  using FeatureAbstract::errorSOUT;

  /*! \name Dealing with the reference value to be reach with this feature.
    @{  */
  DECLARE_NO_REFERENCE();
  /*! @} */

 public:
  FeaturePose( const std::string& name );
  virtual ~FeaturePose( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  /// Computes \f$ {}^oM_{fb} \ominus {}^oM_{fa} {}^{fa}M^*_{fb} \f$
  virtual dg::Vector& computeError( dg::Vector& res,int time );
  /// Computes \f$ \frac{\partial\ominus}{\partial b} {{}^{fb^*}X_{fa}} {}^{fa}\nu^*_{fafb} \f$
  virtual dg::Vector& computeErrorDot( dg::Vector& res,int time );
  /// Computes \f$ \frac{\partial\ominus}{\partial b} {{}^{fb}X_{jb}} {}^{jb}J_{jb} + \frac{\partial\ominus}{\partial a} {{}^{fb^*}X_{ja}} {}^{ja}J_{ja} \f$
  virtual dg::Matrix& computeJacobian( dg::Matrix& res,int time );

  /** Static Feature selection. */
  inline static Flags selectX( void )  { return FLAG_LINE_1; }
  inline static Flags selectY( void )  { return FLAG_LINE_2; }
  inline static Flags selectZ( void )  { return FLAG_LINE_3; }
  inline static Flags selectRX( void ) { return FLAG_LINE_4; }
  inline static Flags selectRY( void ) { return FLAG_LINE_5; }
  inline static Flags selectRZ( void ) { return FLAG_LINE_6; }

  inline static Flags selectTranslation( void ) { return Flags(7); }
  inline static Flags selectRotation( void ) { return Flags(56); }

  virtual void display( std::ostream& os ) const;

 public:
  void servoCurrentPosition( void );
 private:
  Vector7& computeQoMfb (Vector7& res, int time);
  Vector7& computeQoMfbDes (Vector7& res, int time);

  /// \todo Intermediate variables for internal computations
} ;

} /* namespace sot */} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_TRANSFORMATION_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
