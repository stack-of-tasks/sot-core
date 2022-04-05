/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/core/debug.hh>
#include <sot/core/exception-feature.hh>
#include <sot/core/factory.hh>
#include <sot/core/joint-limitator.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(JointLimitator, "JointLimitator");

JointLimitator::JointLimitator(const string &fName)
    : Entity(fName),
      jointSIN(NULL, "JointLimitator(" + name + ")::input(vector)::joint"),
      upperJlSIN(NULL, "JointLimitator(" + name + ")::input(vector)::upperJl"),
      lowerJlSIN(NULL, "JointLimitator(" + name + ")::input(vector)::lowerJl"),
      controlSIN(NULL,
                 "JointLimitator(" + name + ")::input(vector)::controlIN"),
      controlSOUT(boost::bind(&JointLimitator::computeControl, this, _1, _2),
                  jointSIN << upperJlSIN << lowerJlSIN << controlSIN,
                  "JointLimitator(" + name + ")::output(vector)::control"),
      widthJlSINTERN(boost::bind(&JointLimitator::computeWidthJl, this, _1, _2),
                     upperJlSIN << lowerJlSIN,
                     "JointLimitator(" + name + ")::input(vector)::widthJl")

{
  signalRegistration(jointSIN << upperJlSIN << lowerJlSIN << controlSIN
                              << controlSOUT << widthJlSINTERN);
}

dynamicgraph::Vector &JointLimitator::computeWidthJl(dynamicgraph::Vector &res,
                                                     const int &time) {
  sotDEBUGIN(15);

  const dynamicgraph::Vector UJL = upperJlSIN.access(time);
  const dynamicgraph::Vector LJL = lowerJlSIN.access(time);
  const dynamicgraph::Vector::Index SIZE = UJL.size();
  res.resize(SIZE);

  for (unsigned int i = 0; i < SIZE; ++i) {
    res(i) = UJL(i) - LJL(i);
  }

  sotDEBUGOUT(15);
  return res;
}

dynamicgraph::Vector &JointLimitator::computeControl(dynamicgraph::Vector &uOUT,
                                                     int time) {
  sotDEBUGIN(15);

  const dynamicgraph::Vector &q = jointSIN.access(time);
  const dynamicgraph::Vector &UJL = upperJlSIN.access(time);
  const dynamicgraph::Vector &LJL = lowerJlSIN.access(time);
  const dynamicgraph::Vector &uIN = controlSIN.access(time);

  dynamicgraph::Vector::Index controlSize = uIN.size();
  uOUT.resize(controlSize);
  uOUT.setZero();

  dynamicgraph::Vector::Index offset = q.size() - uIN.size();
  assert(offset >= 0);

  for (unsigned int i = 0; i < controlSize; ++i) {
    double qnext = q(i + offset) + uIN(i) * 0.005;
    if ((i + offset < 6) ||  // do not take into account of freeflyer
        ((qnext < UJL(i + offset)) && (qnext > LJL(i + offset)))) {
      uOUT(i) = uIN(i);
    }
    sotDEBUG(25) << i << ": " << qnext << " in? [" << LJL(i) << "," << UJL(i)
                 << "]" << endl;
  }

  sotDEBUGOUT(15);
  return uOUT;
}

void JointLimitator::display(std::ostream &os) const {
  os << "JointLimitator <" << name << "> ... TODO";
}
