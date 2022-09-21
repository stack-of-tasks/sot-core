/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_DERIVATOR_IMPL_H__
#define __SOT_DERIVATOR_IMPL_H__

#include <sot/core/derivator.hh>
#include <sot/core/matrix-geometry.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(derivator_EXPORTS)
#define DERIVATOR_EXPORT __declspec(dllexport)
#else
#define DERIVATOR_EXPORT __declspec(dllimport)
#endif
#else
#define DERIVATOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

#ifdef WIN32
#define DECLARE_SPECIFICATION(className, sotSigType)                \
  class DERIVATOR_EXPORT className : public Derivator<sotSigType> { \
   public:                                                          \
    className(const std::string &name);                             \
  };
#else
#define DECLARE_SPECIFICATION(className, sotSigType) \
  typedef Derivator<sotSigType> className;
#endif

DECLARE_SPECIFICATION(DerivatorDouble, double)
DECLARE_SPECIFICATION(DerivatorVector, dynamicgraph::Vector)
DECLARE_SPECIFICATION(DerivatorMatrix, dynamicgraph::Matrix)
DECLARE_SPECIFICATION(DerivatorVectorQuaternion, VectorQuaternion)
} /* namespace sot */
} /* namespace dynamicgraph */

#endif  // #ifndef __SOT_DERIVATOR_H__
