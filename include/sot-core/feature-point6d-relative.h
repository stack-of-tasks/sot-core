/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      feature-point6d-relative.h
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __SOT_FEATURE_POINT6DRELATIVE_HH__
#define __SOT_FEATURE_POINT6DRELATIVE_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/feature-abstract.h>
#include <sot-core/feature-point6d.h>
#include <sot-core/exception-task.h>
#include <sot-core/vector-utheta.h>

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

namespace sot {
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
  ml::Matrix L;

  

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< MatrixHomogeneous,int > positionReferenceSIN;
  dg::SignalPtr< ml::Matrix,int > articularJacobianReferenceSIN;

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
  /*! The derivative of the error.*/
  dg::SignalTimeDependent<ml::Vector,int> errordotSOUT;
  /*! @} */

 public:
  FeaturePoint6dRelative( const std::string& name );
  virtual ~FeaturePoint6dRelative( void ) {}
  
  virtual ml::Vector& computeError( ml::Vector& res,int time ); 
  virtual ml::Vector& computeErrorDot( ml::Vector& res,int time ); 
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time ); 
  virtual ml::Vector& computeActivation( ml::Vector& res,int time ); 

  virtual void display( std::ostream& os ) const;
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );


} ;

} // namespace sot

#endif // #ifndef __SOT_FEATURE_POINT6DRELATIVE_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
