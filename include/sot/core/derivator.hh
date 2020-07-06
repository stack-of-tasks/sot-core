/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_DERIVATOR_H__
#define __SOT_DERIVATOR_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot/core/flags.hh>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/pool.hh>

/* STD */
#include <string>

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

template <class T> class Derivator : public dynamicgraph::Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

protected:
  T memory;
  bool initialized;
  double timestep;
  static const double TIMESTEP_DEFAULT; //= 1.;

public: /* --- CONSTRUCTION --- */
  static std::string getTypeName(void) { return "Unknown"; }

  Derivator(const std::string &name)
      : dynamicgraph::Entity(name), memory(), initialized(false),
        timestep(TIMESTEP_DEFAULT),
        SIN(NULL, "sotDerivator<" + getTypeName() + ">(" + name + ")::input(" +
                      getTypeName() + ")::sin"),
        SOUT(boost::bind(&Derivator<T>::computeDerivation, this, _1, _2), SIN,
             "sotDerivator<" + getTypeName() + ">(" + name + ")::output(" +
                 getTypeName() + ")::sout"),
        timestepSIN("sotDerivator<" + getTypeName() + ">(" + name +
                    ")::input(double)::dt") {
    signalRegistration(SIN << SOUT << timestepSIN);
    timestepSIN.setReferenceNonConstant(&timestep);
    timestepSIN.setKeepReference(true);
  }

  virtual ~Derivator(void){};

public: /* --- SIGNAL --- */
  dynamicgraph::SignalPtr<T, int> SIN;
  dynamicgraph::SignalTimeDependent<T, int> SOUT;
  dynamicgraph::Signal<double, int> timestepSIN;

protected:
  T &computeDerivation(T &res, int time) {
    if (initialized) {
      res = memory;
      res *= -1;
      memory = SIN(time);
      res += memory;
      if (timestep != 1.)
        res *= (1. / timestep);
    } else {
      initialized = true;
      memory = SIN(time);
      res = memory;
      res *= 0;
    }
    return res;
  }
};
// TODO Derivation of unit quaternion?
template <>
VectorQuaternion &
Derivator<VectorQuaternion>::computeDerivation(VectorQuaternion &res,
                                               int time) {
  if (initialized) {
    res = memory;
    res.coeffs() *= -1;
    memory = SIN(time);
    res.coeffs() += memory.coeffs();
    if (timestep != 1.)
      res.coeffs() *= (1. / timestep);
  } else {
    initialized = true;
    memory = SIN(time);
    res = memory;
    res.coeffs() *= 0;
  }
  return res;
}

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_DERIVATOR_H__
