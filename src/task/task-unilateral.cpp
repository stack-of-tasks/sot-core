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

//#define VP_DEBUG
//#define VP_DEBUG_MODE 15

/* SOT */
#include <sot/core/debug.hh>
#include <sot/core/task-unilateral.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskUnilateral, "TaskUnilateral");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

TaskUnilateral::TaskUnilateral(const std::string &n)
    : Task(n),
      featureList(),
      positionSIN(NULL,
                  "sotTaskUnilateral(" + n + ")::input(vector)::position"),
      referenceInfSIN(
          NULL, "sotTaskUnilateral(" + n + ")::input(vector)::referenceInf"),
      referenceSupSIN(
          NULL, "sotTaskUnilateral(" + n + ")::input(vector)::referenceSup"),
      dtSIN(NULL, "sotTaskUnilateral(" + n + ")::input(double)::dt") {
  taskSOUT.setFunction(
      boost::bind(&TaskUnilateral::computeTaskUnilateral, this, _1, _2));
  taskSOUT.clearDependencies();
  taskSOUT.addDependency(referenceSupSIN);
  taskSOUT.addDependency(referenceInfSIN);
  taskSOUT.addDependency(dtSIN);
  taskSOUT.addDependency(positionSIN);

  signalRegistration(referenceSupSIN << dtSIN << referenceInfSIN
                                     << positionSIN);
}

/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

VectorMultiBound &TaskUnilateral::computeTaskUnilateral(VectorMultiBound &res,
                                                        int time) {
  sotDEBUG(45) << "# In " << getName() << " {" << endl;
  const dynamicgraph::Vector &position = positionSIN(time);
  sotDEBUG(35) << "position = " << position << endl;
  const dynamicgraph::Vector &refInf = referenceInfSIN(time);
  const dynamicgraph::Vector &refSup = referenceSupSIN(time);
  const double &dt = dtSIN(time);
  res.resize(position.size());
  for (unsigned int i = 0; i < res.size(); ++i) {
    MultiBound toto((refInf(i) - position(i)) / dt,
                    (refSup(i) - position(i)) / dt);
    res[i] = toto;
  }

  sotDEBUG(15) << "taskU = " << res << std::endl;
  sotDEBUG(45) << "# Out }" << endl;
  return res;
}

/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

void TaskUnilateral::display(std::ostream &os) const {
  os << "TaskUnilateral " << name << ": " << endl;
}
