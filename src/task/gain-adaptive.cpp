/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* SOT */
#include <sot/core/gain-adaptive.hh>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <dynamic-graph/command-bind.h>

#include <sot/core/debug.hh>
#include <sot/core/exception-signal.hh>
#include <sot/core/factory.hh>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(GainAdaptive, "GainAdaptive");

const double GainAdaptive::ZERO_DEFAULT = .1;
const double GainAdaptive::INFTY_DEFAULT = .1;
const double GainAdaptive::TAN_DEFAULT = 1;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#define __SOT_GAIN_ADAPTATIVE_INIT                                           \
  Entity(name), coeff_a(0), coeff_b(0), coeff_c(0),                          \
      errorSIN(NULL, "sotGainAdaptive(" + name + ")::input(vector)::error"), \
      gainSOUT(boost::bind(&GainAdaptive::computeGain, this, _1, _2),        \
               errorSIN,                                                     \
               "sotGainAdaptive(" + name + ")::output(double)::gain")

void GainAdaptive::addCommands() {
  using namespace ::dynamicgraph::command;
  std::string docstring;
  // Command SetConstant
  docstring =
      "    \n"
      "    setConstant\n"
      "      Input:\n"
      "        floating point value: value at 0. Other values are set to"
      "default.\n"
      "    \n";
  addCommand("setConstant",
             makeCommandVoid1(*this, &GainAdaptive::init, docstring));

  // Command Set
  docstring =
      "    \n"
      "    set\n"
      "      Input:\n"
      "        floating point value: value at 0,\n"
      "        floating point value: value at infinity,\n"
      "        floating point value: value at slope,\n"
      "    \n";
  addCommand("set", makeCommandVoid3(*this, &GainAdaptive::init, docstring));
  docstring =
      "    \n"
      "    set from value at 0 and infinity, with a passing point\n"
      "      Input:\n"
      "        floating point value: value at 0,\n"
      "        floating point value: value at infinity,\n"
      "        floating point value: reference point,\n"
      "        floating point value: percentage at ref point.\n"
      "    \n";
  addCommand(
      "setByPoint",
      makeCommandVoid4(*this, &GainAdaptive::initFromPassingPoint, docstring));
}

GainAdaptive::GainAdaptive(const std::string &name)
    : __SOT_GAIN_ADAPTATIVE_INIT {
  sotDEBUG(15) << "New gain <" << name << ">" << std::endl;
  init();
  Entity::signalRegistration(gainSOUT << errorSIN);
  addCommands();
}

GainAdaptive::GainAdaptive(const std::string &name, const double &lambda)
    : __SOT_GAIN_ADAPTATIVE_INIT {
  init(lambda);
  Entity::signalRegistration(gainSOUT);
  addCommands();
}

GainAdaptive::GainAdaptive(const std::string &name, const double &valueAt0,
                           const double &valueAtInfty, const double &tanAt0)
    : __SOT_GAIN_ADAPTATIVE_INIT {
  init(valueAt0, valueAtInfty, tanAt0);
  Entity::signalRegistration(gainSOUT);
  addCommands();
}

void GainAdaptive::init(const double &valueAt0, const double &valueAtInfty,
                        const double &tanAt0) {
  coeff_a = valueAt0 - valueAtInfty;
  if (0 == coeff_a) {
    coeff_b = 0;
  } else {
    coeff_b = tanAt0 / coeff_a;
  }
  coeff_c = valueAtInfty;

  return;
}

/*
 * The idea is to fix value at 0 and infinity. Now, we are looking for a smart
 * way to chose the slope at 0 or the coeff B (it is more or less the same).
 * I can imagine to way of using a passing point:
 *  - first, imposing a value gref at position xref: g(xref)=gref.
 * In that case, B=-1/xref*log((gref-C)/A):
 *     gnuplot> A=1; C=.1; xref=.1; gref=.4; B=1/xref*log((gref-C)/A)
 *     gnuplot> plot [0:(A+C)/B*10][C-.1*(A-C):A+C+.1*(A-C)] A*exp(-B*x)+C,
 * A+C-B*x
 *  - second solution: imposing to reach a percentage of raise at a given point.
 * It is more or less the same as before, but with gref := C+p*A, for a given p.
 * In that case, B=-1/xref*log(p) gnuplot> A=1; C=.1; xref=.1; p=.1;
 * B=1/xref*log(p) gnuplot> plot [0:(A+C)/B*10][C-.1*(A-C):A+C+.1*(A-C)]
 * A*exp(-B*x)+C, A+C-B*x
 *
 * The second solution is tried in the following.
 */
void GainAdaptive::initFromPassingPoint(const double &valueAt0,
                                        const double &valueAtInfty,
                                        const double &xref,
                                        const double &p)  // gref )
{
  coeff_c = valueAtInfty;
  coeff_a = valueAt0 - valueAtInfty;
  if (0 == coeff_a) {
    coeff_b = 0;
  } else {
    // coeff_b = -1/xref*log( (gref-coeff_c)/coeff_a );
    coeff_b = -1 / xref * log(p);
  }
}

void GainAdaptive::forceConstant(void) { coeff_a = 0; }

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void GainAdaptive::display(std::ostream &os) const {
  os << "Gain Adaptative " << getName();
  try {
    os << " = " << double(gainSOUT.accessCopy());
  } catch (ExceptionSignal e) {
  }
  os << " (" << coeff_a << ";" << coeff_b << ";" << coeff_c << ") ";
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
double &GainAdaptive::computeGain(double &res, int t) {
  sotDEBUGIN(15);
  const dynamicgraph::Vector &error = errorSIN(t);
  const double norm = error.norm();
  res = coeff_a * exp(-coeff_b * norm) + coeff_c;

  sotDEBUGOUT(15);
  return res;
}
