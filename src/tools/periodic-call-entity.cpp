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

/* --- SOT --- */
#include <dynamic-graph/pool.h>

#include <sot/core/debug.hh>
#include <sot/core/factory.hh>
#include <sot/core/periodic-call-entity.hh>

using namespace std;
using namespace dynamicgraph::sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PeriodicCallEntity, "PeriodicCallEntity");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

PeriodicCallEntity::PeriodicCallEntity(const string &fName)
    : Entity(fName),
      PeriodicCall(),
      triger("Tracer(" + fName + ")::triger"),
      trigerOnce("Tracer(" + fName + ")::trigerOnce") {
  signalRegistration(triger << trigerOnce);

  triger.setFunction(
      boost::bind(&PeriodicCallEntity::trigerCall, this, _1, _2));
  trigerOnce.setFunction(
      boost::bind(&PeriodicCallEntity::trigerOnceCall, this, _1, _2));
}

int &PeriodicCallEntity::trigerCall(int &dummy, const int &time) {
  run(time);
  return dummy;
}
int &PeriodicCallEntity::trigerOnceCall(int &dummy, const int &time) {
  run(time);
  clear();
  return dummy;
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void PeriodicCallEntity::display(std::ostream &os) const {
  os << "PeriodicCallEntity <" << name << "> ";
  PeriodicCall::display(os);
}
