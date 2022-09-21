/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef SOT_CORE_DEBUG_HH
#define SOT_CORE_DEBUG_HH
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <sstream>

#include "sot/core/api.hh"

#ifndef VP_DEBUG_MODE
#define VP_DEBUG_MODE 0
#endif  //! VP_DEBUG_MODE

#ifndef VP_TEMPLATE_DEBUG_MODE
#define VP_TEMPLATE_DEBUG_MODE 0
#endif  //! VP_TEMPLATE_DEBUG_MODE

#define SOT_COMMON_TRACES                                       \
  do {                                                          \
    va_list arg;                                                \
    va_start(arg, format);                                      \
    vsnprintf(charbuffer, SIZE, format, arg);                   \
    va_end(arg);                                                \
    outputbuffer << tmpbuffer.str() << charbuffer << std::endl; \
  } while (0)

namespace dynamicgraph {
namespace sot {
class SOT_CORE_EXPORT DebugTrace {
 public:
  static const int SIZE = 512;

  std::stringstream tmpbuffer;
  std::ostream &outputbuffer;
  char charbuffer[SIZE + 1];
  int traceLevel;
  int traceLevelTemplate;

  DebugTrace(std::ostream &os) : outputbuffer(os) {}

  inline void trace(const int level, const char *format, ...) {
    if (level <= traceLevel) SOT_COMMON_TRACES;
    tmpbuffer.str("");
  }

  inline void trace(const char *format, ...) {
    SOT_COMMON_TRACES;
    tmpbuffer.str("");
  }

  inline void trace(const int level = -1) {
    if (level <= traceLevel) outputbuffer << tmpbuffer.str();
    tmpbuffer.str("");
  }

  inline void traceTemplate(const int level, const char *format, ...) {
    if (level <= traceLevelTemplate) SOT_COMMON_TRACES;
    tmpbuffer.str("");
  }

  inline void traceTemplate(const char *format, ...) {
    SOT_COMMON_TRACES;
    tmpbuffer.str("");
  }

  inline DebugTrace &pre(const std::ostream &) { return *this; }

  inline DebugTrace &pre(const std::ostream &, int level) {
    traceLevel = level;
    return *this;
  }

  static const char *DEBUG_FILENAME_DEFAULT;
  static void openFile(const char *filename = DEBUG_FILENAME_DEFAULT);
  static void closeFile(const char *filename = DEBUG_FILENAME_DEFAULT);
};

SOT_CORE_EXPORT extern DebugTrace sotDEBUGFLOW;
SOT_CORE_EXPORT extern DebugTrace sotERRORFLOW;
}  // namespace sot
}  // namespace dynamicgraph

#ifdef VP_DEBUG
#define sotPREDEBUG \
  __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

#define sotPREERROR \
  "\t!! " << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"

#define sotDEBUG(level)                                       \
  if ((level > VP_DEBUG_MODE) ||                              \
      (!dynamicgraph::sot::sotDEBUGFLOW.outputbuffer.good())) \
    ;                                                         \
  else                                                        \
    dynamicgraph::sot::sotDEBUGFLOW.outputbuffer << sotPREDEBUG

#define sotDEBUGMUTE(level)                                   \
  if ((level > VP_DEBUG_MODE) ||                              \
      (!dynamicgraph::sot::sotDEBUGFLOW.outputbuffer.good())) \
    ;                                                         \
  else                                                        \
    dynamicgraph::sot::sotDEBUGFLOW.outputbuffer

#define sotERROR                                            \
  if (!dynamicgraph::sot::sotDEBUGFLOW.outputbuffer.good()) \
    ;                                                       \
  else                                                      \
    dynamicgraph::sot::sotERRORFLOW.outputbuffer << sotPREERROR

#define sotDEBUGF                                                      \
  if (!dynamicgraph::sot::sotDEBUGFLOW.outputbuffer.good())            \
    ;                                                                  \
  else                                                                 \
    dynamicgraph::sot::sotDEBUGFLOW                                    \
        .pre(dynamicgraph::sot::sotDEBUGFLOW.tmpbuffer << sotPREDEBUG, \
             VP_DEBUG_MODE)                                            \
        .trace

#define sotERRORF                                           \
  if (!dynamicgraph::sot::sotDEBUGFLOW.outputbuffer.good()) \
    ;                                                       \
  else                                                      \
    sot::sotERRORFLOW.pre(sot::sotERRORFLOW.tmpbuffer << sotPREERROR).trace

// TEMPLATE
#define sotTDEBUG(level)                                      \
  if ((level > VP_TEMPLATE_DEBUG_MODE) ||                     \
      (!dynamicgraph::sot::sotDEBUGFLOW.outputbuffer.good())) \
    ;                                                         \
  else                                                        \
    dynamicgraph::sot::sotDEBUGFLOW.outputbuffer << sotPREDEBUG

#define sotTDEBUGF                                                     \
  if (!dynamicgraph::sot::sotDEBUGFLOW.outputbuffer.good())            \
    ;                                                                  \
  else                                                                 \
    dynamicgraph::sot::sotDEBUGFLOW                                    \
        .pre(dynamicgraph::sot::sotDEBUGFLOW.tmpbuffer << sotPREDEBUG, \
             VP_TEMPLATE_DEBUG_MODE)                                   \
        .trace

namespace dynamicgraph {
namespace sot {
inline bool sotDEBUG_ENABLE(const int &level) { return level <= VP_DEBUG_MODE; }

inline bool sotTDEBUG_ENABLE(const int &level) {
  return level <= VP_TEMPLATE_DEBUG_MODE;
}
}  // namespace sot
}  // namespace dynamicgraph

/* -------------------------------------------------------------------------- */
#else  // VP_DEBUG
#define sotPREERROR \
  "\t!! " << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") :"
#define sotDEBUG(level) \
  if (1)                \
    ;                   \
  else                  \
    ::dynamicgraph::sot::__null_stream()
#define sotDEBUGMUTE(level) \
  if (1)                    \
    ;                       \
  else                      \
    ::dynamicgraph::sot::__null_stream()
#define sotERROR sotERRORFLOW.outputbuffer << sotPREERROR

namespace dynamicgraph {
namespace sot {
inline void sotDEBUGF(const int, const char *, ...) {}
inline void sotDEBUGF(const char *, ...) {}
inline void sotERRORF(const int, const char *, ...) {}
inline void sotERRORF(const char *, ...) {}
inline std::ostream &__null_stream() {
  // This function should never be called. With -O3,
  // it should not appear in the generated binary.
  static std::ostream os(NULL);
  return os;
}
}  // namespace sot
}  // namespace dynamicgraph

// TEMPLATE
#define sotTDEBUG(level) \
  if (1)                 \
    ;                    \
  else                   \
    ::dynamicgraph::sot::__null_stream()

namespace dynamicgraph {
namespace sot {
inline void sotTDEBUGF(const int, const char *, ...) {}
inline void sotTDEBUGF(const char *, ...) {}
}  // namespace sot
}  // namespace dynamicgraph

#define sotDEBUG_ENABLE(level) false
#define sotTDEBUG_ENABLE(level) false

#endif  // VP_DEBUG

#define sotDEBUGIN(level) sotDEBUG(level) << "# In {" << std::endl
#define sotDEBUGOUT(level) sotDEBUG(level) << "# Out }" << std::endl
#define sotDEBUGINOUT(level) sotDEBUG(level) << "# In/Out { }" << std::endl

#define sotTDEBUGIN(level) sotTDEBUG(level) << "# In {" << std::endl
#define sotTDEBUGOUT(level) sotTDEBUG(level) << "# Out }" << std::endl
#define sotTDEBUGINOUT(level) sotTDEBUG(level) << "# In/Out { }" << std::endl

#endif  //! #ifdef SOT_CORE_DEBUG_HH

// Local variables:
// c-basic-offset: 2
// End:
