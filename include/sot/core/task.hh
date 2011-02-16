/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_TASK_H__
#define __SOT_TASK_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* STD */
#include <string>

/* SOT */
#include <sot/core/feature-abstract.hh>
#include <sot/core/flags.hh>
#include <sot/core/task-abstract.hh>

#include <sot/core/exception-task.hh>

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

namespace dynamicgraph { namespace sot {
namespace dg = dynamicgraph;

class SOTTASK_EXPORT Task
: public TaskAbstract
{
 public:
  typedef std::list< FeatureAbstract* > FeatureList_t;
 protected:
  FeatureList_t featureList;

 private: //HACK
  static const std::string CLASS_NAME;
 public: //HACK
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  Task( const std::string& n );

  void addFeature( FeatureAbstract& s );
  void clearFeatureList( void );
  FeatureList_t & getFeatureList( void ) { return featureList; }

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

} /* namespace sot */} /* namespace dynamicgraph */



#endif /* #ifndef __SOT_TASK_H__ */


