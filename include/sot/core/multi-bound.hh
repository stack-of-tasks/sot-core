/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_MultiBound_H__
#define __SOT_MultiBound_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <string>
#include <vector>

/* SOT */
#include "sot/core/api.hh"
#include <dynamic-graph/signal-caster.h>
#include <sot/core/exception-task.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

class SOT_CORE_EXPORT MultiBound {
public:
  enum MultiBoundModeType { MODE_SINGLE, MODE_DOUBLE };
  enum SupInfType { BOUND_SUP, BOUND_INF };

public: // protected:
  MultiBoundModeType mode;
  double boundSingle;
  double boundSup, boundInf;
  bool boundSupSetup, boundInfSetup;

public:
  MultiBound(const double x = 0.);
  MultiBound(const double xi, const double xs);
  MultiBound(const double x, const SupInfType bound);
  MultiBound(const MultiBound &clone);

public: // Acessors
  MultiBoundModeType getMode(void) const;
  double getSingleBound(void) const;
  double getDoubleBound(const SupInfType bound) const;
  bool getDoubleBoundSetup(const SupInfType bound) const;

public: // Modifiors
  void setDoubleBound(SupInfType boundType, double boundValue);
  void unsetDoubleBound(SupInfType boundType);
  void setSingleBound(double boundValue);

public:
  SOT_CORE_EXPORT friend std::ostream &operator<<(std::ostream &os,
                                                  const MultiBound &m);
  SOT_CORE_EXPORT friend std::istream &operator>>(std::istream &is,
                                                  MultiBound &m);
};

/* --------------------------------------------------------------------- */
typedef std::vector<MultiBound> VectorMultiBound;
SOT_CORE_EXPORT std::ostream &operator<<(std::ostream &os,
                                         const VectorMultiBound &v);
SOT_CORE_EXPORT std::istream &operator>>(std::istream &os, VectorMultiBound &v);

} /* namespace sot */

template <>
struct signal_io<sot::MultiBound> : signal_io_unimplemented<sot::MultiBound> {};
} /* namespace dynamicgraph */

#endif // #ifndef __SOT_MultiBound_H__
