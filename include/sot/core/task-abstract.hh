/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_TASKABSTRACT_H__
#define __SOT_TASKABSTRACT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <Eigen/SVD>
#include <dynamic-graph/linear-algebra.h>

/* STD */
#include <string>

/* SOT */
#include "sot/core/api.hh"
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot/core/multi-bound.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/// Hierarchical element of the stack of tasks.
///
/// A task computes a value and a Jacobian as output signals.
/// Once stacked into a solver, the solver will compute the control
/// vector that makes the task values converge toward zero in the
/// order defined by the priority levels.
///
/// \image html pictures/task.png "Task diagram: Task types derive from
/// TaskAbstract. The value and Jacobian of a Task are computed from the
/// features that are stored in the task.

class SOT_CORE_EXPORT TaskAbstract : public dynamicgraph::Entity {
public:
  /* Use a derivative of this class to store computational memory. */
  class MemoryTaskAbstract {
  public:
    int timeLastChange;

  public:
    MemoryTaskAbstract(void) : timeLastChange(0){};
    virtual ~MemoryTaskAbstract(void){};

  public:
    virtual void display(std::ostream &os) const = 0;
    friend std::ostream &operator<<(std::ostream &os,
                                    const MemoryTaskAbstract &tcm) {
      tcm.display(os);
      return os;
    }
  };

public:
  MemoryTaskAbstract *memoryInternal;

protected:
  void taskRegistration(void);

public:
  TaskAbstract(const std::string &n);

public: /* --- SIGNALS --- */
  dynamicgraph::SignalTimeDependent<VectorMultiBound, int> taskSOUT;
  dynamicgraph::SignalTimeDependent<dynamicgraph::Matrix, int> jacobianSOUT;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_TASKABSTRACT_H__ */
