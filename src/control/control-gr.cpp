/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* SOT */

#include <sot/core/debug.hh>
class ControlGR__INIT {
 public:
  ControlGR__INIT(void) { dynamicgraph::sot::DebugTrace::openFile(); }
};
ControlGR__INIT ControlGR_initiator;
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <dynamic-graph/factory.h>

#include <sot/core/binary-op.hh>
#include <sot/core/control-gr.hh>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ControlGR, "ControlGR");

const double ControlGR::TIME_STEP_DEFAULT = .001;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#define __SOT_ControlGR_INIT

ControlGR::ControlGR(const std::string &name)
    : Entity(name),
      TimeStep(0),
      matrixASIN(NULL, "ControlGR(" + name + ")::input(matrix)::matrixA"),
      accelerationSIN(NULL,
                      "ControlGR(" + name + ")::input(vector)::acceleration"),
      gravitySIN(NULL, "ControlGR(" + name + ")::input(vector)::gravity"),
      controlSOUT(boost::bind(&ControlGR::computeControl, this, _1, _2),
                  matrixASIN << accelerationSIN << gravitySIN,
                  "ControlGR(" + name + ")::output(vector)::control") {
  init(TimeStep);
  Entity::signalRegistration(matrixASIN << accelerationSIN << gravitySIN
                                        << controlSOUT);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void ControlGR::init(const double &Stept) {
  TimeStep = Stept;

  return;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void ControlGR::display(std::ostream &os) const {
  os << "ControlGR " << getName();
  try {
    os << "control = " << controlSOUT;
  } catch (ExceptionSignal e) {
  }
  os << " (" << TimeStep << ") ";
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

double &ControlGR::setsize(int dimension)

{
  _dimension = dimension;
  return _dimension;
}

dynamicgraph::Vector &ControlGR::computeControl(dynamicgraph::Vector &tau,
                                                int t) {
  sotDEBUGIN(15);

  const dynamicgraph::Matrix &matrixA = matrixASIN(t);
  const dynamicgraph::Vector &acceleration = accelerationSIN(t);
  const dynamicgraph::Vector &gravity = gravitySIN(t);
  dynamicgraph::Vector::Index size = acceleration.size();
  tau.resize(size);
  // tau*=0;
  /* for(unsigned i = 0u; i < size; ++i)
     {
         tp = gravity(i);
         tau(i) = acceleration;
         tau(i) *= matrixA;
         tau(i) += tp;
     }*/

  tau = matrixA * acceleration;
  sotDEBUG(15) << "torque = A*ddot(q)= " << matrixA * acceleration << std::endl;
  tau += gravity;

  /*
  tau(1) *= 0;
  tau(7) *= 0;
  tau(24) *=0;
  tau(17)=0;*/

  sotDEBUG(15) << "matrixA =" << matrixA << std::endl;
  sotDEBUG(15) << "acceleration =" << acceleration << std::endl;
  sotDEBUG(15) << "gravity =" << gravity << std::endl;
  sotDEBUG(15) << "gravity compensation torque =" << tau << std::endl;
  sotDEBUGOUT(15);

  return tau;
}
