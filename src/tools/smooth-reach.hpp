/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/core/smooth-reach.hh>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SmoothReach, "SmoothReach");

SmoothReach::SmoothReach(const std::string &name)
    : Entity(name)

      ,
      start(0), goal(0), startTime(-1), lengthTime(-1), isStarted(false),
      isParam(true)

      ,
      startSIN(NULL, "SmoothReach(" + name + ")::input(vector)::start"),
      goalSOUT(boost::bind(&SmoothReach::goalSOUT_function, this, _1, _2),
               sotNOSIGNAL "SmoothReach(" + name + ")::output(vector)::goal") {
  sotDEBUGIN(5);

  signalRegistration(startSIN << goalSOUT);

  sotDEBUGOUT(5);
}

double smoothFunction(double x) { return x; }

dynamicgraph::Vector &SmoothReach::goalSOUT_function(dynamicgraph::Vector &goal,
                                                     const int &time) {
  if (isParam) {
    start = startSIN(time);
    startTime = time;

    assert(start.size() == goal.size());
    isParam = false;
    isStarted = true;
  }

  if (isReady) {
    double x = double(time - start) / length;
    if (x > 1)
      x = 1;
    double x1 = smoothFunction(x);
    double x0 = 1 - x1;
    goal = start * x0 + goal * x1;
  }

  return goal;
}

void SmoothReach::gset(const dynamicgraph::Vector &goalDes,
                       const int &lengthDes) {
  goal = goalDes;
  length = lengthDes;
  isParam = true;
}

const dynamicgraph::Vector &SmoothReach::getGoal(void) { return goal; }

const int &SmoothReach::getLength(void) { return length; }

const int &SmoothReach::getStart(void) { return start; }
