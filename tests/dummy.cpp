/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <iostream>
#include <sot/core/debug.hh>

int main(int, char **) {
  dynamicgraph::sot::sotDEBUGFLOW.openFile();
  dynamicgraph::sot::sotDEBUGFLOW.trace("test test test");

  std::cout << "It works!" << std::endl;
  return 0;
}
