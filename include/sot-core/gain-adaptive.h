/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      task.h
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



#ifndef __SOT_GAIN_ADAPTATIVE_HH__
#define __SOT_GAIN_ADAPTATIVE_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotGainAdaptative_EXPORTS)
#    define SOTGAINADAPTATIVE_EXPORT __declspec(dllexport)
#  else  
#    define SOTGAINADAPTATIVE_EXPORT  __declspec(dllimport)
#  endif 
#else
#  define SOTGAINADAPTATIVE_EXPORT 
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTGAINADAPTATIVE_EXPORT sotGainAdaptative
: public Entity
{

 public: /* --- CONSTANTS --- */

  /* Default values. */
  static const double ZERO_DEFAULT;   // = 0.1
  static const double INFTY_DEFAULT;  // = 0.1
  static const double TAN_DEFAULT;    // = 1.

 public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display( std::ostream& os ) const; 
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }


 protected: 
  
  /* Parameters of the adaptative-gain function: 
   * lambda (x) = a * exp (-b*x) + c. */
  double coeff_a;
  double coeff_b;
  double coeff_c;

 public: /* --- CONSTRUCTORS ---- */

  sotGainAdaptative( const std::string & name );
  sotGainAdaptative( const std::string & name,const double& lambda );
  sotGainAdaptative( const std::string & name,
		     const double& valueAt0, 
		     const double& valueAtInfty,
		     const double& tanAt0 );

 public: /* --- INIT --- */

  inline void init( void ) { init( ZERO_DEFAULT,INFTY_DEFAULT,TAN_DEFAULT ); }
  inline void init( const double& lambda ) { init( lambda,lambda,1.); }
  void init( const double& valueAt0, 
	     const double& valueAtInfty,
	     const double& tanAt0 );
  void forceConstant( void );
    
 public:  /* --- SIGNALS --- */
  SignalPtr<ml::Vector,int> errorSIN;
  SignalTimeDependant<double,int> gainSOUT;
 protected:
  double& computeGain( double& res,int t );

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
			    std::ostream& os );
};





#endif // #ifndef __SOT_GAIN_ADAPTATIVE_HH__
