/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <sot/core/debug.hh>
#include <sot/core/smooth-reach.hh>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SmoothReach, "SmoothReach");

SmoothReach::SmoothReach(const std::string &name)
    : Entity(name)

      ,
      start(0u),
      goal(0u),
      startTime(-1),
      lengthTime(-1),
      isStarted(false),
      isParam(true)

      ,
      smoothMode(2),
      smoothParam(1.2)

      ,
      startSIN(NULL, "SmoothReach(" + name + ")::input(vector)::start"),
      goalSOUT(boost::bind(&SmoothReach::goalSOUT_function, this, _1, _2),
               sotNOSIGNAL, "SmoothReach(" + name + ")::output(vector)::goal")

{
  sotDEBUGIN(5);

  signalRegistration(startSIN << goalSOUT);
  initCommands();
  goalSOUT.setNeedUpdateFromAllChildren(true);
  sotDEBUGOUT(5);
}

void SmoothReach::initCommands(void) {
  using namespace command;
  addCommand("set",
             makeCommandVoid2(*this, &SmoothReach::set,
                              docCommandVoid2("Set the curve.", "vector (goal)",
                                              "int (duration)")));
  addCommand("param",
             makeCommandVoid2(*this, &SmoothReach::setSmoothing,
                              docCommandVoid2("Set the parameter.",
                                              "int (mode)", "double (beta)")));
}

double SmoothReach::smoothFunction(double x) {
  switch (smoothMode) {
    case 0:
      return x;

    case 1: {
      // const double smoothParam = 0.45;
      return tanh(-smoothParam / x + smoothParam / (1 - x)) / 2 + 0.5;
    }
    case 2: {
      // const double smoothParam = 1.5;
      return atan(-smoothParam / x + smoothParam / (1 - x)) / M_PI + 0.5;
    }
  }
  return 0;
}

void SmoothReach::setSmoothing(const int &mode, const double &param) {
  smoothMode = mode;
  smoothParam = param;
}

dynamicgraph::Vector &SmoothReach::goalSOUT_function(dynamicgraph::Vector &res,
                                                     const int &time) {
  if (isParam) {
    start = startSIN(time);
    startTime = time;

    assert(start.size() == goal.size());
    isParam = false;
    isStarted = true;
  }

  if (isStarted) {
    double x = double(time - startTime) / lengthTime;
    if (x > 1) x = 1;
    double x1 = smoothFunction(x);
    double x0 = 1 - x1;
    res = start * x0 + goal * x1;
  }

  return res;
}

void SmoothReach::set(const dynamicgraph::Vector &goalDes,
                      const int &lengthDes) {
  goal = goalDes;
  lengthTime = lengthDes;
  isParam = true;
}

const dynamicgraph::Vector &SmoothReach::getGoal(void) { return goal; }

const int &SmoothReach::getLength(void) { return lengthTime; }

const int &SmoothReach::getStart(void) { return startTime; }

void SmoothReach::display(std::ostream &os) const {
  os << "Status: " << isStarted << isParam << std::endl
     << "Goal: " << goal << "start: " << start << "Times: " << startTime << " "
     << lengthTime << std::endl;
}
