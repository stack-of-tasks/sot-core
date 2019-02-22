/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FEATURE_POINT6DRELATIVE_HH__
#define __SOT_FEATURE_POINT6DRELATIVE_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/feature-point6d.hh>
#include <sot/core/exception-task.hh>
#include <sot/core/matrix-geometry.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_point6d_relative_EXPORTS)
#    define SOTFEATUREPOINT6DRELATIVE_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATUREPOINT6DRELATIVE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATUREPOINT6DRELATIVE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeaturePoint6dRelative
  \brief Class that defines the motion of a point of the body wrt. another
  point.
*/
class SOTFEATUREPOINT6DRELATIVE_EXPORT FeaturePoint6dRelative
: public FeaturePoint6d
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  dg::Matrix L;



  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< MatrixHomogeneous,int > positionReferenceSIN;
  dg::SignalPtr< dg::Matrix,int > articularJacobianReferenceSIN;

  /*! dg::Signals related to the computation of the derivative of
    the error
  @{ */

  /*! dg::Signals giving the derivative of the input signals.
    @{*/
  /*! Derivative of the relative position. */
  dg::SignalPtr< MatrixHomogeneous,int > dotpositionSIN;
  /*! Derivative of the reference position. */
  dg::SignalPtr< MatrixHomogeneous,int > dotpositionReferenceSIN;
  /*! @} */

  using FeaturePoint6d::getReference;

 public:
  FeaturePoint6dRelative( const std::string& name );
  virtual ~FeaturePoint6dRelative( void ) {}

  virtual dg::Vector& computeError( dg::Vector& res,int time );
  virtual dg::Vector& computeErrorDot( dg::Vector& res,int time );
  virtual dg::Matrix& computeJacobian( dg::Matrix& res,int time );

  virtual void display( std::ostream& os ) const;

  void initCommands( void );
  void initSdes( const std::string& featureDesiredName );

} ;

} /* namespace sot */} /* namespace dynamicgraph */

#endif // #ifndef __SOT_FEATURE_POINT6DRELATIVE_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
