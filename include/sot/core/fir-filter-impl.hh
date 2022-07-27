/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FIRFILTER_IMPL_HH__
#define __SOT_FIRFILTER_IMPL_HH__

#include <sot/core/fir-filter.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(fir_filter_EXPORTS)
#define FIL_FILTER_EXPORT __declspec(dllexport)
#else
#define FIL_FILTER_EXPORT __declspec(dllimport)
#endif
#else
#define FIL_FILTER_EXPORT
#endif

#ifdef WIN32
#define DECLARE_SPECIFICATION(className, sotSigType, sotCoefType) \
  class FIL_FILTER_EXPORT className                               \
      : public FIRFilter<sotSigType, sotCoefType> {               \
   public:                                                        \
    className(const std::string &name);                           \
  };
#else
#define DECLARE_SPECIFICATION(className, sotSigType, sotCoefType) \
  typedef FIRFilter<sotSigType, sotCoefType> className;
#endif  // WIN32

namespace dynamicgraph {
namespace sot {

DECLARE_SPECIFICATION(FIRFilterDoubleDouble, double, double)
DECLARE_SPECIFICATION(FIRFilterVectorDouble, Vector, double)
DECLARE_SPECIFICATION(FIRFilterVectorMatrix, Vector, Matrix)

}  // namespace sot
}  // namespace dynamicgraph
#endif
