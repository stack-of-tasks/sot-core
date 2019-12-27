/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_CONTIIFSTREAM_HH__
#define __SOT_CONTIIFSTREAM_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <fstream>
#include <sstream>
#ifndef WIN32
#include <unistd.h>
#endif
#include <list>

#include "sot/core/api.hh"
#ifndef WIN32
#include <pthread.h>
#endif

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */
class SOT_CORE_EXPORT Contiifstream {
protected:
  std::string filename;
  std::streamoff cursor;
  static const unsigned int BUFFER_SIZE = 256;
  char buffer[BUFFER_SIZE];
  std::list<std::string> reader;
  bool first;

public: /* --- Constructor --- */
  Contiifstream(const std::string &n = "");
  ~Contiifstream(void);
  void open(const std::string &n) {
    filename = n;
    cursor = 0;
  }

public: /* --- READ FILE --- */
  bool loop(void);

public: /* --- READ LIST --- */
  inline bool ready(void) { return 0 < reader.size(); }
  std::string next(void);
};

} /* namespace sot */
} /* namespace dynamicgraph */

#endif /* #ifndef __SOT_CONTIIFSTREAM_HH__ */
