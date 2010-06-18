/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      task-conti.h
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



#ifndef __SOT_TASKCONTI_H__
#define __SOT_TASKCONTI_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* STD */
#include <string>

/* SOT */
#include <sot-core/feature-abstract.h>
#include <sot-core/flags.h>
#include <sot-core/task.h>

#include <sot-core/exception-task.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (sotTaskConti_EXPORTS)
#    define SOTTASKCONTI_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKCONTI_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKCONTI_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

class SOTTASKCONTI_EXPORT sotTaskConti
: public sotTask
{
 protected:
  enum TimeRefValues
    {
      TIME_REF_UNSIGNIFICANT = -1
      ,TIME_REF_TO_BE_SET =-2
    };


  int timeRef;
  double mu;
  ml::Vector q0;

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  sotTaskConti( const std::string& n );

  void referenceTime( const unsigned int & t ) { timeRef = t; }
  const int &  referenceTime( void ) { return timeRef; }

  /* --- COMPUTATION --- */
  sotVectorMultiBound& computeContiDesiredVelocity( sotVectorMultiBound &task,
                                                    const int & time );


  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  SignalPtr< ml::Vector,int > controlPrevSIN;

  /* --- DISPLAY ------------------------------------------------------------ */
  void display( std::ostream& os ) const;

  /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
};

} // namespace sot



#endif /* #ifndef __SOT_TASKCONTI_H__ */


