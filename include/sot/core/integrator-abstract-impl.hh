/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_INTEGRATOR_ABSTRACT_VECTOR_H__
#define __SOT_INTEGRATOR_ABSTRACT_VECTOR_H__

/* --- SOT PLUGIN  --- */
#include <sot/core/integrator-abstract.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(integrator_abstract_EXPORTS)
#define INTEGRATOR_ABSTRACT_EXPORT __declspec(dllexport)
#else
#define INTEGRATOR_ABSTRACT_EXPORT __declspec(dllimport)
#endif
#else
#define INTEGRATOR_ABSTRACT_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#ifdef WIN32
#define DECLARE_SPECIFICATION(className, sotSigType, sotCoefType)              \
  class INTEGRATOR_ABSTRACT_EXPORT className                                   \
      : public IntegratorAbstract<sotSigType, sotCoefType> {                   \
  public:                                                                      \
    className(const std::string &name);                                        \
  };
#else
#define DECLARE_SPECIFICATION(className, sotSigType, sotCoefType)              \
  typedef IntegratorAbstract<sotSigType, sotCoefType> className;
#endif

namespace dynamicgraph {
namespace sot {
DECLARE_SPECIFICATION(IntegratorAbstractDouble, double, double)
DECLARE_SPECIFICATION(IntegratorAbstractVector, dynamicgraph::Vector,
                      dynamicgraph::Matrix)
} // namespace sot
} // namespace dynamicgraph
#endif // #ifndef  __SOT_MAILBOX_HH
