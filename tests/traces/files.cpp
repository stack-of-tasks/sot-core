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

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

using namespace std;

int main() {

  boost::filesystem::create_directory("foobar");
  ofstream file("foobar/cheeze");
  file << "tastes good!\n";
  file.close();
  if (!boost::filesystem::exists("foobar/cheeze"))
    std::cout << "Something is rotten in foobar\n";

  return 0;
}
