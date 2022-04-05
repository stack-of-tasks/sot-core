/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/core/multi-bound.hh>
//#define VP_DEBUG_MODE 25
#include <sot/core/debug.hh>

using namespace dynamicgraph::sot;

MultiBound::MultiBound(const double x)
    : mode(MODE_SINGLE),
      boundSingle(x),
      boundSup(0),
      boundInf(0),
      boundSupSetup(false),
      boundInfSetup(false) {}

MultiBound::MultiBound(const double xi, const double xs)
    : mode(MODE_DOUBLE),
      boundSingle(0),
      boundSup(xs),
      boundInf(xi),
      boundSupSetup(true),
      boundInfSetup(true) {}

MultiBound::MultiBound(const double x, const MultiBound::SupInfType bound)
    : mode(MODE_DOUBLE),
      boundSingle(0),
      boundSup((bound == BOUND_SUP) ? x : 0),
      boundInf((bound == BOUND_INF) ? x : 0),
      boundSupSetup(bound == BOUND_SUP),
      boundInfSetup(bound == BOUND_INF) {}

MultiBound::MultiBound(const MultiBound &clone)
    : mode(clone.mode),
      boundSingle(clone.boundSingle),
      boundSup(clone.boundSup),
      boundInf(clone.boundInf),
      boundSupSetup(clone.boundSupSetup),
      boundInfSetup(clone.boundInfSetup) {}

MultiBound::MultiBoundModeType MultiBound::getMode(void) const { return mode; }
double MultiBound::getSingleBound(void) const {
  if (MODE_SINGLE != mode) {
    SOT_THROW ExceptionTask(ExceptionTask::BOUND_TYPE,
                            "Accessing single bound of a non-single type.");
  }
  return boundSingle;
}
double MultiBound::getDoubleBound(const MultiBound::SupInfType bound) const {
  if (MODE_DOUBLE != mode) {
    SOT_THROW ExceptionTask(ExceptionTask::BOUND_TYPE,
                            "Accessing double bound of a non-double type.");
  }
  switch (bound) {
    case BOUND_SUP: {
      if (!boundSupSetup) {
        SOT_THROW ExceptionTask(ExceptionTask::BOUND_TYPE,
                                "Accessing un-setup sup bound.");
      }
      return boundSup;
    }
    case BOUND_INF: {
      if (!boundInfSetup) {
        SOT_THROW ExceptionTask(ExceptionTask::BOUND_TYPE,
                                "Accessing un-setup inf bound");
      }
      return boundInf;
    }
  }
  return 0;
}
bool MultiBound::getDoubleBoundSetup(const MultiBound::SupInfType bound) const {
  if (MODE_DOUBLE != mode) {
    SOT_THROW ExceptionTask(ExceptionTask::BOUND_TYPE,
                            "Accessing double bound of a non-double type.");
  }
  switch (bound) {
    case BOUND_SUP:
      return boundSupSetup;
    case BOUND_INF:
      return boundInfSetup;
  }
  return false;
}
void MultiBound::setDoubleBound(SupInfType boundType, double boundValue) {
  if (MODE_DOUBLE != mode) {
    mode = MODE_DOUBLE;
    boundSupSetup = false;
    boundInfSetup = false;
  }
  switch (boundType) {
    case BOUND_INF:
      boundInfSetup = true;
      boundInf = boundValue;
      break;
    case BOUND_SUP:
      boundSupSetup = true;
      boundSup = boundValue;
      break;
  }
}
void MultiBound::unsetDoubleBound(SupInfType boundType) {
  if (MODE_DOUBLE != mode) {
    mode = MODE_DOUBLE;
    boundSupSetup = false;
    boundInfSetup = false;
  } else {
    switch (boundType) {
      case BOUND_INF:
        boundInfSetup = false;
        break;
      case BOUND_SUP:
        boundSupSetup = false;
        break;
    }
  }
}
void MultiBound::setSingleBound(double boundValue) {
  mode = MODE_SINGLE;
  boundSingle = boundValue;
}

inline static void SOT_MULTI_BOUND_CHECK_C(std::istream &is, char check,
                                           VectorMultiBound &v) {
  char c;
  is.get(c);
  if (c != check) {
    v.resize(0);
    sotERROR << "Error while parsing vector multi-bound. Waiting for a '"
             << check << "'. Get '" << c << "' instead. " << std::endl;
    SOT_THROW ExceptionTask(ExceptionTask::PARSER_MULTI_BOUND,
                            "Error parsing vector multi-bound.");
  }
}

namespace dynamicgraph {
namespace sot {

std::ostream &operator<<(std::ostream &os, const MultiBound &m) {
  switch (m.mode) {
    case MultiBound::MODE_SINGLE: {
      os << m.boundSingle;
      break;
    }
    case MultiBound::MODE_DOUBLE: {
      os << "(";
      if (m.boundInfSetup)
        os << m.boundInf;
      else
        os << "--";
      os << ",";
      if (m.boundSupSetup)
        os << m.boundSup;
      else
        os << "--";
      os << ")";
      break;
    }
  }
  return os;
}

std::istream &operator>>(std::istream &is, MultiBound &m) {
  sotDEBUGIN(15);
  char c;
  double val;
  is.get(c);
  if (c == '(') {
    sotDEBUG(15) << "Double" << std::endl;
    char c2[3];
    is.get(c2, 3);
    if (std::string(c2, 2) != "--") {
      is.putback(c2[1]);
      is.putback(c2[0]);
      //{char strbuf[256]; is.getline(strbuf,256); sotDEBUG(1) <<
      //"#"<<strbuf<<"#"<<std::endl;}
      is >> val;
      sotDEBUG(15) << "First val = " << val << std::endl;
      m.setDoubleBound(MultiBound::BOUND_INF, val);
    } else {
      m.unsetDoubleBound(MultiBound::BOUND_INF);
    }
    is.get(c);
    if (c != ',') {
      sotERROR << "Error while parsing multi-bound. Waiting for a ','. Get '"
               << c << "' instead. " << std::endl;
      SOT_THROW ExceptionTask(
          ExceptionTask::PARSER_MULTI_BOUND,
          "Error parsing multi-bound, while waiting for a ','.");
    }

    is.get(c2, 3);
    if (std::string(c2, 2) != "--") {
      is.putback(c2[1]);
      is.putback(c2[0]);
      is >> val;
      sotDEBUG(15) << "Second val = " << val << std::endl;
      m.setDoubleBound(MultiBound::BOUND_SUP, val);
    } else {
      m.unsetDoubleBound(MultiBound::BOUND_SUP);
    }
    is.get(c);
    if (c != ')') {
      sotERROR << "Error while parsing multi-bound. Waiting for a ')'. Get '"
               << c << "' instead. " << std::endl;
      SOT_THROW ExceptionTask(
          ExceptionTask::PARSER_MULTI_BOUND,
          "Error parsing multi-bound, while waiting for a ')'.");
    }
  } else {
    sotDEBUG(15) << "Single ('" << c << "')" << std::endl;
    is.putback(c);
    is >> val;
    m.setSingleBound(val);
  }

  sotDEBUGOUT(15);
  return is;
}

std::ostream &operator<<(std::ostream &os, const VectorMultiBound &v) {
  os << "[" << v.size() << "](";
  for (VectorMultiBound::const_iterator iter = v.begin(); iter != v.end();
       ++iter) {
    if (iter != v.begin()) os << ",";
    os << (*iter);
  }
  return os << ")";
}

std::istream &operator>>(std::istream &is, VectorMultiBound &v) {
  unsigned int vali;

  /* Read the vector size. */
  SOT_MULTI_BOUND_CHECK_C(is, '[', v);
  is >> vali;
  v.resize(vali);
  SOT_MULTI_BOUND_CHECK_C(is, ']', v);

  /* Loop for the vals. */
  SOT_MULTI_BOUND_CHECK_C(is, '(', v);
  for (unsigned int i = 0; i < vali; ++i) {
    is >> v[i];
    if (i != vali - 1) {
      SOT_MULTI_BOUND_CHECK_C(is, ',', v);
    } else {
      SOT_MULTI_BOUND_CHECK_C(is, ')', v);
    }
  }

  return is;
}

} /* namespace sot */
} /* namespace dynamicgraph */
