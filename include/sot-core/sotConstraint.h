/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotConstraint.h
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



#ifndef __SOT_CONSTRAINT_H__
#define __SOT_CONSTRAINT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* STD */
#include <string>

/* SOT */
#include <sot-core/sotFeatureAbstract.h>
#include <sot-core/sotFlags.h>
#include <sot-core/sotTaskAbstract.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/exception-task.h>
#include <sot-core/exception-signal.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (sotConstraint_EXPORTS)
#    define SOTCONSTRAINT_EXPORT __declspec(dllexport)
#  else  
#    define SOTCONSTRAINT_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTCONSTRAINT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */



class SOTCONSTRAINT_EXPORT sotConstraint
: public sotTaskAbstract
{
 protected:
  typedef std::list< Signal<ml::Matrix,int>* > JacobianList;
  JacobianList jacobianList;
  
 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  sotConstraint( const std::string& n );

  void addJacobian( Signal<ml::Matrix,int>& sig );
  void clearJacobianList( void );

  void setControlSelection( const sotFlags& act );
  void addControlSelection( const sotFlags& act );
  void clearControlSelection( void );

  /* --- COMPUTATION --- */
  ml::Matrix& computeJacobian( ml::Matrix& J,int time );

  /* --- DISPLAY ------------------------------------------------------------ */
  SOTCONSTRAINT_EXPORT friend std::ostream& operator<< ( std::ostream& os,const sotConstraint& t );

  /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine
			    ,std::istringstream& cmdArgs
			    ,std::ostream& os );
};





#endif /* #ifndef __SOT_CONSTRAINT_H__ */


