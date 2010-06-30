/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      feature-line-distance.h
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


#ifndef __SOT_FEATURE_LINEDISTANCE_HH__
#define __SOT_FEATURE_LINEDISTANCE_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/feature-abstract.h>
#include <sot-core/exception-task.h>
#include <sot-core/matrix-homogeneous.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (feature_line_distance_EXPORTS)
#    define SOTFEATURELINEDISTANCE_EXPORT __declspec(dllexport)
#  else
#    define SOTFEATURELINEDISTANCE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFEATURELINEDISTANCE_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

/*!
  \class FeatureLineDistance
  \brief Class that defines point-3d control feature
*/
class SOTFEATURELINEDISTANCE_EXPORT FeatureLineDistance
: public FeatureAbstract
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< MatrixHomogeneous,int > positionSIN;
  dg::SignalPtr< ml::Matrix,int > articularJacobianSIN;
  dg::SignalPtr< ml::Vector,int > positionRefSIN;
  dg::SignalPtr< ml::Vector,int > vectorSIN;
  dg::SignalTimeDependent<ml::Vector,int> lineSOUT;

  using FeatureAbstract::desiredValueSIN;
  using FeatureAbstract::selectionSIN;

  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::errorSOUT;
  using FeatureAbstract::activationSOUT;

 public:
  FeatureLineDistance( const std::string& name );
  virtual ~FeatureLineDistance( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );

  virtual ml::Vector& computeError( ml::Vector& res,int time );
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time );
  virtual ml::Vector& computeActivation( ml::Vector& res,int time );
  ml::Vector& computeLineCoordinates( ml::Vector& cood,int time );

  virtual void display( std::ostream& os ) const;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );

} ;


} // namespace sot

#endif // #ifndef __SOT_FEATURE_LINEDISTANCE_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
