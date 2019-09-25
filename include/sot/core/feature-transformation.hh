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
  \class FeaturePoint6d
  \brief Class that defines point-6d control feature
*/
class SOT_CORE_DLLAPI FeatureTransformation
  : public FeatureAbstract
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  /*! \name Input signals
    @{
  */
  /// Input transformation of <em>Joint A</em> wrt to world frame.
  dg::SignalPtr< MatrixHomogeneous, int > oMja;
  /// Input transformation of <em>Frame A</em> wrt to <em>Joint A</em>.
  dg::SignalPtr< MatrixHomogeneous, int > jaMfa;
  /// Input transformation of <em>Joint B</em> wrt to world frame.
  dg::SignalPtr< MatrixHomogeneous, int > oMjb;
  /// Input transformation of <em>Frame B</em> wrt to <em>Joint B</em>.
  dg::SignalPtr< MatrixHomogeneous, int > jbMfb;
  /// Jacobians of the input <em>Joint A</em>.
  /// \todo explicit the convention (local frame)
  dg::SignalPtr< Matrix,int > jaJja;
  /// Jacobians of the input <em>Joint B</em>.
  /// \todo explicit the convention (local frame)
  dg::SignalPtr< Matrix,int > jbJjb;

  /// The desired pose of <em>Frame B</em> wrt to <em>Frame A</em>.
  dg::SignalPtr< MatrixHomogeneous, int > faMfbDes;
  /// The desired velocity of <em>Frame B</em> wrt to <em>Frame A</em>.
  /// \todo explicit in which frame this velocity is expressed.
  dg::SignalPtr< Vector, int > faMfbDesDot;
  /*! @} */

  /*! \name Output signals
    @{
  */
  /// Transformation of <em>Frame B</em> wrt to <em>Frame A</em>.
  /// It is expressed as a translation followed by a quaternion.
  SignalTimeDependent< Vector7, int > faMfb;

  /// The Vector7 version of faMfbDes, for internal purposes.
  SignalTimeDependent< Vector7, int > faMfbDes_q;
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
  FeatureTransformation( const std::string& name );
  virtual ~FeatureTransformation( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  virtual dg::Vector& computeError( dg::Vector& res,int time );
  virtual dg::Vector& computeErrorDot( dg::Vector& res,int time );
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
  Vector7& computefaMfb (Vector7& res, int time);
  Vector7& computefaMfbDes_q (Vector7& res, int time);

  /// \todo Intermediate variables for internal computations
} ;

} /* namespace sot */} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_TRANSFORMATION_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
