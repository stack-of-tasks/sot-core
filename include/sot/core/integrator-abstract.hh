/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_INTEGRATOR_ABSTRACT_H__
#define __SOT_INTEGRATOR_ABSTRACT_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/pool.h>
#include <sot/core/debug.hh>
#include <sot/core/flags.hh>

/* STD */
#include <string>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

/*! \brief integrates an ODE. If Y is the output and X the input, the
 * following equation is integrated:
 * a_p * d(p)Y / dt^p + .... + a_0 Y = b_m * d(m)X / dt^m + ... . b_0 X
 * a_i are the coefficients of the denominator of the associated transfer
 * function between X and Y, while the b_i are those of the numerator.
 */
template <class sigT, class coefT>
class IntegratorAbstract : public dynamicgraph::Entity {
public:
  IntegratorAbstract(const std::string &name)
      : dynamicgraph::Entity(name),
        SIN(NULL, "sotIntegratorAbstract(" + name + ")::input(vector)::sin"),
        SOUT(boost::bind(&IntegratorAbstract<sigT, coefT>::integrate, this, _1,
                         _2),
             SIN, "sotIntegratorAbstract(" + name + ")::output(vector)::sout") {
    signalRegistration(SIN << SOUT);

    using namespace dynamicgraph::command;

    const std::string typeName =
        Value::typeName(dynamicgraph::command::ValueHelper<coefT>::TypeID);

    addCommand(
        "pushNumCoef",
        makeCommandVoid1(
            *this, &IntegratorAbstract::pushNumCoef,
            docCommandVoid1("Push a new numerator coefficient", typeName)));
    addCommand(
        "pushDenomCoef",
        makeCommandVoid1(
            *this, &IntegratorAbstract::pushDenomCoef,
            docCommandVoid1("Push a new denomicator coefficient", typeName)));

    addCommand(
        "popNumCoef",
        makeCommandVoid0(*this, &IntegratorAbstract::popNumCoef,
                         docCommandVoid0("Pop a new numerator coefficient")));
    addCommand(
        "popDenomCoef",
        makeCommandVoid0(*this, &IntegratorAbstract::popDenomCoef,
                         docCommandVoid0("Pop a new denomicator coefficient")));
  }

  virtual ~IntegratorAbstract() {}

  virtual sigT &integrate(sigT &res, int time) = 0;

public:
  void pushNumCoef(const coefT &numCoef) { numerator.push_back(numCoef); }
  void pushDenomCoef(const coefT &denomCoef) {
    denominator.push_back(denomCoef);
  }
  void popNumCoef() { numerator.pop_back(); }
  void popDenomCoef() { denominator.pop_back(); }

public:
  dynamicgraph::SignalPtr<sigT, int> SIN;

  dynamicgraph::SignalTimeDependent<sigT, int> SOUT;

protected:
  std::vector<coefT> numerator;
  std::vector<coefT> denominator;
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif
