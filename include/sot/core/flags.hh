/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_FLAGS_H
#define __SOT_FLAGS_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <ostream>
#include <vector>

/* SOT */
#include "sot/core/api.hh"
#include <dynamic-graph/signal-caster.h>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
namespace sot {

class SOT_CORE_EXPORT Flags {
protected:
  std::vector<bool> flags;
  bool outOfRangeFlag;

public:
  Flags(const bool &b = false);
  Flags(const char *flags);
  Flags(const std::vector<bool> &flags);

  void add(const bool &b);

  Flags operator!(void) const;
  SOT_CORE_EXPORT friend Flags operator&(const Flags &f1, const Flags &f2);
  SOT_CORE_EXPORT friend Flags operator|(const Flags &f1, const Flags &f2);
  Flags &operator&=(const Flags &f2);
  Flags &operator|=(const Flags &f2);

  SOT_CORE_EXPORT friend std::ostream &operator<<(std::ostream &os,
                                                  const Flags &fl);
  SOT_CORE_EXPORT friend std::istream &operator>>(std::istream &is, Flags &fl);
  bool operator()(const int &i) const;

  operator bool(void) const;

  void unset(const unsigned int &i);
  void set(const unsigned int &i);
};

} // namespace sot

template <>
struct signal_io<sot::Flags> : signal_io_unimplemented<sot::Flags> {};
} // namespace dynamicgraph

#endif /* #ifndef __SOT_FLAGS_H */
