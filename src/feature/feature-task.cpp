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

/* --- SOT --- */
#include <dynamic-graph/pool.h>

#include <sot/core/debug.hh>
#include <sot/core/exception-feature.hh>
#include <sot/core/feature-task.hh>
#include <sot/core/task.hh>
using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeatureTask, "FeatureTask");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

FeatureTask::FeatureTask(const string &pointName) : FeatureGeneric(pointName) {}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void FeatureTask::display(std::ostream &os) const {
  os << "Feature from task <" << getName();
  if (taskPtr) os << ": from task " << taskPtr->getName();
  os << std::endl;
}
