/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      JointLimitator.h
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


#ifndef __SOT_FEATURE_JOINTLIMITS_HH__
#define __SOT_FEATURE_JOINTLIMITS_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <sot-core/exception-task.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/sot-core-api.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


namespace sot {

namespace dg = dynamicgraph;

/*!
  \class JointLimitator
*/
class SOT_CORE_EXPORT JointLimitator
: public dg::Entity
{

 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  
  double threshold;
  const static double THRESHOLD_DEFAULT; // = .99;

  /* --- SIGNALS ------------------------------------------------------------ */
 public:

  dg::SignalPtr< ml::Vector,int > jointSIN;
  dg::SignalPtr< ml::Vector,int > upperJlSIN;
  dg::SignalPtr< ml::Vector,int > lowerJlSIN;
  dg::SignalPtr< ml::Vector,int > controlSIN;
  dg::SignalTimeDependent< ml::Vector,int > controlSOUT;
  dg::SignalTimeDependent< ml::Vector,int > widthJlSINTERN;

 public:
  JointLimitator( const std::string& name );
  virtual ~JointLimitator( void ) {}

  virtual ml::Vector& computeControl( ml::Vector& res,int time ); 
  ml::Vector& computeWidthJl( ml::Vector& res,const int& time );

  virtual void display( std::ostream& os ) const;
  void commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );
} ;


} // namespace sot



#endif // #ifndef __SOT_FEATURE_JOINTLIMITS_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
