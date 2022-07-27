/*
 * Copyright 2018,
 * Julian Viereck
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <boost/function.hpp>
#include <sot/core/factory.hh>
#include <sot/core/gradient-ascent.hh>

namespace dg = ::dynamicgraph;

/* ---------------------------------------------------------------------------*/
/* ------- GENERIC HELPERS -------------------------------------------------- */
/* ---------------------------------------------------------------------------*/

namespace dynamicgraph {
namespace sot {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(GradientAscent, "GradientAscent");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

GradientAscent::GradientAscent(const std::string &n)
    : Entity(n),
      gradientSIN(NULL, "GradientAscent(" + n + ")::input(vector)::gradient"),
      learningRateSIN(NULL,
                      "GradientAscent(" + n + ")::input(double)::learningRate"),
      refresherSINTERN("GradientAscent(" + n + ")::intern(dummy)::refresher"),
      valueSOUT(boost::bind(&GradientAscent::update, this, _1, _2),
                gradientSIN << refresherSINTERN,
                "GradientAscent(" + n + ")::output(vector)::value"),
      init(false) {
  // Register signals into the entity.
  signalRegistration(gradientSIN << learningRateSIN << valueSOUT);
  refresherSINTERN.setDependencyType(TimeDependency<int>::ALWAYS_READY);
}

GradientAscent::~GradientAscent() {}

/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */

dynamicgraph::Vector &GradientAscent::update(dynamicgraph::Vector &res,
                                             const int &inTime) {
  const dynamicgraph::Vector &gradient = gradientSIN(inTime);
  const double &learningRate = learningRateSIN(inTime);

  if (init == false) {
    init = true;
    value = gradient;
    value.setZero();
    res.resize(value.size());
  }

  value += learningRate * gradient;
  res = value;
  return res;
}

} /* namespace sot */
} /* namespace dynamicgraph */
