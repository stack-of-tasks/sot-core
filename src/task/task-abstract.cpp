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
#include <sot/core/pool.hh>
#include <sot/core/task-abstract.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

TaskAbstract::TaskAbstract(const std::string &n)
    : Entity(n),
      memoryInternal(NULL),
      taskSOUT("sotTaskAbstract(" + n + ")::output(vector)::task"),
      jacobianSOUT("sotTaskAbstract(" + n + ")::output(matrix)::jacobian") {
  taskRegistration();
  signalRegistration(taskSOUT << jacobianSOUT);
}

void TaskAbstract::taskRegistration(void) {
  PoolStorage::getInstance()->registerTask(name, this);
}
