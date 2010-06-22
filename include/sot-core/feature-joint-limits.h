/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      feature-joint-limits.h
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

/* SOT */
#include <sot-core/feature-abstract.h>
#include <sot-core/exception-task.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotFeatureJointLimits_EXPORTS)
#    define SOTFEATUREJOINTLIMITS_EXPORT __declspec(dllexport)
#  else  
#    define SOTFEATUREJOINTLIMITS_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTFEATUREJOINTLIMITS_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

/*!
  \class FeatureJointLimits
  \brief Class that defines gradient vector for jl avoidance.
*/
class SOTFEATUREJOINTLIMITS_EXPORT FeatureJointLimits 
: public FeatureAbstract
{

 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:
  
  double threshold;
  const static double THRESHOLD_DEFAULT; // = .9;

/*   unsigned int freeFloatingIndex,freeFloatingSize; */
/*   static const unsigned int FREE_FLOATING_INDEX = 0; */
/*   static const unsigned int FREE_FLOATING_SIZE = 5; */

  /* --- SIGNALS ------------------------------------------------------------ */
 public:

  SignalPtr< ml::Vector,int > jointSIN;
  SignalPtr< ml::Vector,int > upperJlSIN;
  SignalPtr< ml::Vector,int > lowerJlSIN;
  SignalTimeDependant< ml::Vector,int > widthJlSINTERN;

  using FeatureAbstract::selectionSIN;

  using FeatureAbstract::jacobianSOUT;
  using FeatureAbstract::errorSOUT;
  using FeatureAbstract::activationSOUT;

 public:
  FeatureJointLimits( const std::string& name );
  virtual ~FeatureJointLimits( void ) {}

  virtual unsigned int& getDimension( unsigned int & dim, int time );
  
  virtual ml::Vector& computeError( ml::Vector& res,int time ); 
  virtual ml::Matrix& computeJacobian( ml::Matrix& res,int time ); 
  virtual ml::Vector& computeActivation( ml::Vector& res,int time ); 
  ml::Vector& computeWidthJl( ml::Vector& res,const int& time );

  /** Static Feature selection. */
  inline static Flags selectActuated( void ); 
  
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
