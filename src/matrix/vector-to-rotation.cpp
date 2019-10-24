/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/core/vector-to-rotation.hh>

#include <sot/core/debug.hh>
#include <sot/core/factory.hh>
#include <sot/core/macros-signal.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VectorToRotation, "VectorToRotation");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

VectorToRotation::VectorToRotation(const std::string &name)
    : Entity(name), size(0), axes(0),
      SIN(NULL, "sotVectorToRotation(" + name + ")::output(vector)::sin"),
      SOUT(SOT_MEMBER_SIGNAL_1(VectorToRotation::computeRotation, SIN,
                               dynamicgraph::Vector),
           "sotVectorToRotation(" + name + ")::output(matrixRotation)::sout") {

  signalRegistration(SIN << SOUT);
}
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

MatrixRotation &
VectorToRotation::computeRotation(const dynamicgraph::Vector &angles,
                                  MatrixRotation &res) {
  res.setIdentity();
  MatrixRotation Ra, Rtmp;
  for (unsigned int i = 0; i < size; ++i) {
    Ra.setIdentity();
    const double ca = cos(angles(i));
    const double sa = sin(angles(i));
    const unsigned int i_X = 0, i_Y = 1, i_Z = 2;
    switch (axes[i]) {
    case AXIS_X: {
      Ra(i_Y, i_Y) = ca;
      Ra(i_Y, i_Z) = -sa;
      Ra(i_Z, i_Y) = sa;
      Ra(i_Z, i_Z) = ca;
      break;
    }
    case AXIS_Y: {
      Ra(i_Z, i_Z) = ca;
      Ra(i_Z, i_X) = -sa;
      Ra(i_X, i_Z) = sa;
      Ra(i_X, i_X) = ca;
      break;
    }
    case AXIS_Z: {
      Ra(i_X, i_X) = ca;
      Ra(i_X, i_Y) = -sa;
      Ra(i_Y, i_X) = sa;
      Ra(i_Y, i_Y) = ca;
      break;
    }
    }

    sotDEBUG(15) << "R" << i << " = " << Ra;
    Rtmp = res * Ra;
    res = Rtmp;
  }

  return res;
}
