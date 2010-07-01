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
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef __SOT_TASK_H__
#define __SOT_TASK_H__

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
#include <sot-core/task-abstract.h>

#include <sot-core/exception-task.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined task_EXPORTS
#    define SOTTASK_EXPORT __declspec(dllexport)
#  else
#    define SOTTASK_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASK_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/*!
  @ingroup tasks
  This class defines the basic elements of a task.
  A task is defined as \f$ {\bf s}  ={\bf e}({\bf q}) \f$
  where \f${\bf s} \f$ is a set of features and \f${\bf q}\f$ the
  actuated joints of the robot. <br>
  It is assume that \f$ {\bf e} = - \lambda \dot{\bf e} \f$.
  Moreover as it assumed that this task can provide:
  \f$ {\bf J} = \frac{\delta f}{\delta {\bf q}} \f$
  It then possible to compute
  \f$ \dot{\bf q} = -\lambda {\bf J}^{\#} \dot{\bf e}\f$
  with \f$ \dot{\bf e} = {\bf s}^{des} - {\bf s}^* \f$,
  and \f$ {\bf s}^{des}\f$ the desired feature and
  \f$ {\bf s}^* \f$ the one currently measured.

  It is possible to add features or clear the list of features.
  This class makes also possible to select some of the
  listed of features to compute the control law through setControlSelection,
  addControlSelection, clearControlSelection.


 */

namespace sot {
namespace dg = dynamicgraph;

class SOTTASK_EXPORT Task
: public TaskAbstract
{
 protected:
  std::list< FeatureAbstract* > featureList;

 public:
 private: //HACK
  static const std::string CLASS_NAME;
 public: //HACK
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  Task( const std::string& n );

  void addFeature( FeatureAbstract& s );
  void clearFeatureList( void );

  void setControlSelection( const Flags& act );
  void addControlSelection( const Flags& act );
  void clearControlSelection( void );

  /* --- COMPUTATION --- */
  ml::Vector& computeError( ml::Vector& error,int time );
  VectorMultiBound&
    computeTaskExponentialDecrease( VectorMultiBound& errorRef,int time );
  ml::Matrix& computeJacobian( ml::Matrix& J,int time );
  ml::Vector& computeFeatureActivation( ml::Vector& h,int time );


  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dg::SignalPtr< double,int > controlGainSIN;
  dg::SignalPtr< double,int > dampingGainSINOUT;
  dg::SignalPtr< Flags,int > controlSelectionSIN; // At the task level or at the feature level?

 public:
  dg::SignalTimeDependent< ml::Vector,int > errorSOUT;

  /* --- DISPLAY ------------------------------------------------------------ */
  void display( std::ostream& os ) const;
  //  friend std::ostream& operator<< ( std::ostream& os,const Task& t );

  /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine
			    ,std::istringstream& cmdArgs
			    ,std::ostream& os );
  /* --- Writing graph --- */
  virtual std::ostream& writeGraph( std::ostream& os ) const;
};

} // namespace sot



#endif /* #ifndef __SOT_TASK_H__ */


