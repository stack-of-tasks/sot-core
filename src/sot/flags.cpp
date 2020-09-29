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

/*! System framework */
#include <list>
#include <stdlib.h>

/*! Local Framework */
#include <sot/core/debug.hh>
#include <sot/core/flags.hh>

using namespace std;
using namespace dynamicgraph::sot;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

Flags::Flags(const bool &b) : flags(), outOfRangeFlag(b) {}

Flags::Flags(const char *_flags)
    : flags(strlen(_flags)), outOfRangeFlag(false) {
  for (unsigned int i = 0; i < flags.size(); ++i) {
    switch (_flags[i]) {
    case '0':
      flags[i] = false;
      break;
    case '1':
      flags[i] = true;
      break;
    case ' ':
      break;
    default:
      throw std::invalid_argument("Could not parse input string " +
                                  std::string(_flags) + ". Expected 0 or 1.");
    }
  }
}

Flags::Flags(const std::vector<bool> &_flags)
    : flags(_flags), outOfRangeFlag(false) {}

Flags::operator bool(void) const {
  if (outOfRangeFlag)
    return true;
  for (unsigned int i = 0; i < flags.size(); ++i)
    if (flags[i])
      return true;
  return false;
}

/* --------------------------------------------------------------------- */

bool Flags::operator()(const int &i) const {
  if (i < (int)flags.size())
    return flags[i];
  return outOfRangeFlag;
}

/* --------------------------------------------------------------------- */
void Flags::add(const bool &b) { flags.push_back(b); }

/* --------------------------------------------------------------------- */
void Flags::set(const unsigned int &idx) {
  if (idx < flags.size())
    flags[idx] = true;
}

void Flags::unset(const unsigned int &idx) {
  if (idx < flags.size())
    flags[idx] = false;
}

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
Flags Flags::operator!(void) const {
  Flags res = *this;
  res.flags.flip();
  res.outOfRangeFlag = !outOfRangeFlag;
  return res;
}

Flags operator&(const Flags &f1, const Flags &f2) {
  Flags res = f1;
  res &= f2;
  return res;
}

Flags operator|(const Flags &f1, const Flags &f2) {
  Flags res = f1;
  res |= f2;
  return res;
}

Flags &Flags::operator&=(const Flags &f2) {
  if (f2.flags.size() > flags.size())
    flags.resize(f2.flags.size(), outOfRangeFlag);
  for (unsigned int i = 0; i < f2.flags.size(); ++i)
    flags[i] = flags[i] & f2.flags[i];
  for (auto i = f2.flags.size(); i < flags.size(); ++i)
    flags[i] = flags[i] & f2.outOfRangeFlag;
  outOfRangeFlag = outOfRangeFlag && f2.outOfRangeFlag;
  return *this;
}

Flags &Flags::operator|=(const Flags &f2) {
  if (f2.flags.size() > flags.size())
    flags.resize(f2.flags.size(), outOfRangeFlag);
  for (unsigned int i = 0; i < f2.flags.size(); ++i)
    flags[i] = flags[i] | f2.flags[i];
  for (auto i = f2.flags.size(); i < flags.size(); ++i)
    flags[i] = flags[i] | f2.outOfRangeFlag;
  outOfRangeFlag = outOfRangeFlag || f2.outOfRangeFlag;
  return *this;
}

/* --------------------------------------------------------------------- */
std::ostream &operator<<(std::ostream &os, const Flags &fl) {
  for (auto f : fl.flags)
    os << (f ? '1' : '0');
  return os;
}

std::istream &operator>>(std::istream &is, Flags &fl) {
  char c;
  fl.flags.clear();
  while (is.get(c).good()) {
    switch (c) {
    case '0':
      fl.flags.push_back(false);
      break;
    case '1':
      fl.flags.push_back(true);
      break;
    case ' ':
      break;
    default:
      throw std::invalid_argument("Could not parse input character " +
                                  std::string(1, c) + ". Expected 0 or 1.");
    }
  }
  return is;
}

} /* namespace sot */
} /* namespace dynamicgraph */
