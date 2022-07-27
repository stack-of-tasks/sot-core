/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <sot/core/contiifstream.hh>
#include <sot/core/debug.hh>

using namespace dynamicgraph::sot;

Contiifstream::Contiifstream(const std::string &n)
    : filename(n), cursor(0), first(true) {}

Contiifstream::~Contiifstream(void) { sotDEBUGINOUT(5); }

bool Contiifstream::loop(void) {
  sotDEBUGIN(25);
  bool res = false;

  std::fstream file(filename.c_str());

  file.seekg(cursor);
  file.sync();

  while (1) {
    file.get(buffer, BUFFER_SIZE);
    if (file.gcount()) {
      res = true;
      std::string line(buffer);
      if (!first) reader.push_back(line);
      cursor = file.tellg();
      cursor++;
      file.get(*buffer);  // get the last char ( = '\n')
      sotDEBUG(15) << "line: " << line << std::endl;
    } else {
      break;
    }
  }

  first = false;
  sotDEBUGOUT(25);
  return res;
}

std::string Contiifstream::next(void) {
  std::string res = *reader.begin();
  reader.pop_front();
  return res;
}
