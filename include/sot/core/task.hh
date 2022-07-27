/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_TASK_H__
#define __SOT_TASK_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* STD */
#include <string>

/* SOT */
#include <sot/core/exception-task.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/flags.hh>
#include <sot/core/task-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined task_EXPORTS
#define SOTTASK_EXPORT __declspec(dllexport)
#else
#define SOTTASK_EXPORT __declspec(dllimport)
#endif
#else
#define SOTTASK_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/*!
  @ingroup tasks
  @class dynamicgraph::sot::Task task.hh "Definition"
  @brief Class that defines the basic elements of a task.

  A task is defined as \f$ {\bf s}  ={\bf e}({\bf q}) \f$
  where \f${\bf s} \f$ is a set of features and \f${\bf q}\f$ the
  actuated joints of the robot. <br>
  It is assumes that \f$ \dot{\bf e} = - \lambda {\bf e} \f$.
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

namespace dynamicgraph {
namespace sot {

class SOTTASK_EXPORT Task : public TaskAbstract {
 public:
  typedef std::list<FeatureAbstract *> FeatureList_t;

 protected:
  FeatureList_t featureList;
  bool withDerivative;

  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  Task(const std::string &n);
  void initCommands(void);

  void addFeature(FeatureAbstract &s);
  void addFeatureFromName(const std::string &name);
  void clearFeatureList(void);
  FeatureList_t &getFeatureList(void) { return featureList; }

  void setControlSelection(const Flags &act);
  void addControlSelection(const Flags &act);
  void clearControlSelection(void);

  void setWithDerivative(const bool &s);
  bool getWithDerivative(void);

  /* --- COMPUTATION --- */
  dynamicgraph::Vector &computeError(dynamicgraph::Vector &error, int time);
  VectorMultiBound &computeTaskExponentialDecrease(VectorMultiBound &errorRef,
                                                   int time);
  dynamicgraph::Matrix &computeJacobian(dynamicgraph::Matrix &J, int time);
  dynamicgraph::Vector &computeErrorTimeDerivative(dynamicgraph::Vector &res,
                                                   int time);

  /* --- SIGNALS ------------------------------------------------------------ */
 public:
  dynamicgraph::SignalPtr<double, int> controlGainSIN;
  dynamicgraph::SignalPtr<double, int> dampingGainSINOUT;
  dynamicgraph::SignalPtr<Flags, int> controlSelectionSIN;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> errorSOUT;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
      errorTimeDerivativeSOUT;

  /* --- DISPLAY ------------------------------------------------------------ */
  void display(std::ostream &os) const;

  /* --- Writing graph --- */
  virtual std::ostream &writeGraph(std::ostream &os) const;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_TASK_H__ */
