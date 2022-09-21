/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <iostream>
//#include <sot/core/sot-h.hh>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/debug.hh>
#include <sot/core/feature-abstract.hh>
#include <sot/core/feature-visual-point.hh>
#include <sot/core/gain-adaptive.hh>
#include <sot/core/task.hh>

using namespace std;
using namespace dynamicgraph::sot;

double drand(void) { return 2 * ((double)rand()) / RAND_MAX - 1; }
dynamicgraph::Matrix &mrand(dynamicgraph::Matrix &J) {
  for (int i = 0; i < J.rows(); ++i)
    for (int j = 0; j < J.cols(); ++j) J(i, j) = drand();
  return J;
}

int main(void) {
  sotDEBUGF("# In {");

  srand(12);
  dynamicgraph::Matrix Jq(6, 6);
  Jq.setIdentity();

  dynamicgraph::Vector p1xy(2);
  p1xy(0) = 1.;
  p1xy(1) = -2;

  sotDEBUGF("Create feature");
  FeatureVisualPoint *p1 = new FeatureVisualPoint("p1");
  FeatureVisualPoint *p1des = new FeatureVisualPoint("p1des");

  p1->articularJacobianSIN.setReference(&Jq);
  p1->selectionSIN = Flags(true);
  p1->setReference(p1des);
  p1->xySIN = p1xy;

  p1des->xySIN = dynamicgraph::Vector(2);

  sotDEBUGF("Create Task");
  //  sotDEBUG(0) << dynamicgraph::MATLAB;

  Task *task = new Task("task");
  task->addFeature(*p1);
  task->addFeature(*p1);

  GainAdaptive *lambda = new GainAdaptive("g");
  lambda->errorSIN.plug(&task->errorSOUT);

  task->controlGainSIN.plug(&lambda->gainSOUT);
  task->dampingGainSINOUT = .1;
  task->controlSelectionSIN = Flags(true);

  task->jacobianSOUT.display(cout) << endl;
  task->jacobianSOUT.displayDependencies(cout) << endl;

  sotDEBUG(0) << "J" << task->jacobianSOUT(2);
  sotDEBUG(0) << "e" << task->errorSOUT(2) << endl;

  sotDEBUGF("# Out }");

  return 0;
}
