/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOTVECTORTOMATRIX_HH
#define __SOTVECTORTOMATRIX_HH

#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot/core/matrix-geometry.hh>

/* Matrix */
#include <dynamic-graph/linear-algebra.h>

/* STD */
#include <vector>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(vector_to_rotation_EXPORTS)
#define SOTVECTORTOROTATION_EXPORT __declspec(dllexport)
#else
#define SOTVECTORTOROTATION_EXPORT __declspec(dllimport)
#endif
#else
#define SOTVECTORTOROTATION_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace dynamicgraph {
namespace sot {

class SOTVECTORTOROTATION_EXPORT VectorToRotation
    : public dynamicgraph::Entity {
  enum sotAxis { AXIS_X, AXIS_Y, AXIS_Z };

  unsigned int size;
  std::vector<sotAxis> axes;

public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  VectorToRotation(const std::string &name);

  virtual ~VectorToRotation(void) {}

  dynamicgraph::SignalPtr<dynamicgraph::Vector, int> SIN;
  dynamicgraph::SignalTimeDependent<MatrixRotation, int> SOUT;

  MatrixRotation &computeRotation(const dynamicgraph::Vector &angles,
                                  MatrixRotation &res);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif // #ifndef __SOTVECTORTOMATRIX_HH
