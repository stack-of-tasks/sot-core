/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/all-commands.h>

#include <sot/core/debug.hh>
#include <sot/core/task-pd.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#include <sot/core/factory.hh>

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskPD, "TaskPD");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

TaskPD::TaskPD(const std::string &n)
    : Task(n),
      previousError(),
      beta(1),
      errorDotSOUT(boost::bind(&TaskPD::computeErrorDot, this, _1, _2),
                   errorSOUT,
                   "sotTaskPD(" + n + ")::output(vector)::errorDotOUT"),
      errorDotSIN(NULL, "sotTaskPD(" + n + ")::input(vector)::errorDot") {
  taskSOUT.setFunction(boost::bind(&TaskPD::computeTaskModif, this, _1, _2));
  taskSOUT.addDependency(errorDotSOUT);

  signalRegistration(errorDotSOUT << errorDotSIN);
  initCommand();
  errorDotSIN.plug(&errorDotSOUT);
}

/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

dynamicgraph::Vector &TaskPD::computeErrorDot(dynamicgraph::Vector &errorDot,
                                              int time) {
  sotDEBUG(15) << "# In {" << endl;

  const dynamicgraph::Vector &errCur = errorSOUT(time);
  if (previousError.size() == errCur.size()) {
    errorDot = errCur;
    errorDot -= previousError;
    previousError = errCur;
  } else {
    errorDot.resize(errCur.size());
    errorDot.setZero();
    previousError = errCur;
  }
  sotDEBUG(15) << "# Out }" << endl;
  return errorDot;
}

VectorMultiBound &TaskPD::computeTaskModif(VectorMultiBound &task, int time) {
  sotDEBUG(15) << "# In {" << endl;

  const dynamicgraph::Vector &errorDot = errorDotSIN(time);
  Task::computeTaskExponentialDecrease(task, time);

  sotDEBUG(25) << " Task = " << task;
  sotDEBUG(25) << " edot = " << errorDot;

  for (unsigned int i = 0; i < task.size(); ++i) {
    task[i] = task[i].getSingleBound() - (beta * errorDot(i));
  }

  sotDEBUG(15) << "# Out }" << endl;
  return task;
}

/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
#include <sot/core/pool.hh>

void TaskPD::initCommand(void) {
  using namespace command;
  addCommand("setBeta",
             makeDirectSetter(*this, &beta, docDirectSetter("beta", "double")));
}
