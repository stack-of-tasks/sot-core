/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_INTEGRATOR_EULER_IMPL_H__
#define __SOT_INTEGRATOR_EULER_IMPL_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/integrator-euler.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(integrator_euler_EXPORTS)
#define INTEGRATOR_EULER_EXPORT __declspec(dllexport)
#else
#define INTEGRATOR_EULER_EXPORT __declspec(dllimport)
#endif
#else
#define INTEGRATOR_EULER_EXPORT
#endif

#ifdef WIN32
#define DECLARE_SPECIFICATION(className, sotSigType, sotCoefType) \
  class INTEGRATOR_EULER_EXPORT className                         \
      : public IntegratorEuler<sotSigType, sotCoefType> {         \
   public:                                                        \
    std::string getTypeName(void);                                \
    className(const std::string &name);                           \
  };
#else
#define DECLARE_SPECIFICATION(className, sotSigType, sotCoefType) \
  typedef IntegratorEuler<sotSigType, sotCoefType> className;
#endif  // WIN32

namespace dynamicgraph {
namespace sot {
DECLARE_SPECIFICATION(IntegratorEulerDoubleDouble, double, double)
DECLARE_SPECIFICATION(IntegratorEulerVectorDouble, Vector, double)
DECLARE_SPECIFICATION(IntegratorEulerVectorMatrix, Vector, Matrix)
}  // namespace sot
}  // namespace dynamicgraph

#endif
