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

/* SOT */
#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/factory.h>

#include <boost/bind.hpp>
#include <sot/core/debug.hh>
#include <sot/core/reader.hh>
#include <sstream>

using namespace dynamicgraph;
using namespace dynamicgraph::sot;
using namespace std;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(sotReader, "Reader");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

sotReader::sotReader(const std::string n)
    : Entity(n),
      selectionSIN(NULL, "Reader(" + n + ")::input(flag)::selec"),
      vectorSOUT(boost::bind(&sotReader::getNextData, this, _1, _2),
                 sotNOSIGNAL, "Reader(" + n + ")::vector"),
      matrixSOUT(boost::bind(&sotReader::getNextMatrix, this, _1, _2),
                 vectorSOUT, "Reader(" + n + ")::matrix"),
      dataSet(),
      currentData(),
      iteratorSet(false),
      rows(0),
      cols(0) {
  signalRegistration(selectionSIN << vectorSOUT << matrixSOUT);
  selectionSIN = true;
  vectorSOUT.setNeedUpdateFromAllChildren(true);

  initCommands();
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void sotReader::load(const string &filename) {
  sotDEBUGIN(15);

  std::ifstream datafile(filename.c_str());
  const unsigned int SIZE = 1024;
  char buffer[SIZE];
  std::vector<double> newline;
  while (datafile.good()) {
    datafile.getline(buffer, SIZE);
    const unsigned int gcount = (unsigned int)(datafile.gcount());
    if (gcount >= SIZE) { /* TODO read error, line to long. */
    }
    std::istringstream iss(buffer);
    newline.clear();
    sotDEBUG(25) << "Get line = '" << buffer << "'" << std::endl;
    while (1) {
      double x;
      iss >> x;
      if (!iss.fail())
        newline.push_back(x);
      else
        break;
      sotDEBUG(45) << "New data = " << x << std::endl;
    }
    if (newline.size() > 0) dataSet.push_back(newline);
  }

  sotDEBUGOUT(15);
}

void sotReader::clear(void) {
  sotDEBUGIN(15);

  dataSet.clear();
  iteratorSet = false;

  sotDEBUGOUT(15);
}

void sotReader::rewind(void) {
  sotDEBUGIN(15);
  iteratorSet = false;
  sotDEBUGOUT(15);
}

dynamicgraph::Vector &sotReader::getNextData(dynamicgraph::Vector &res,
                                             const unsigned int time) {
  sotDEBUGIN(15);

  if (!iteratorSet) {
    sotDEBUG(15) << "Start the list" << std::endl;
    currentData = dataSet.begin();
    iteratorSet = true;
  } else if (currentData != dataSet.end()) {
    ++currentData;
  }

  if (currentData == dataSet.end()) {
    sotDEBUGOUT(15);
    return res;
  }

  const Flags &selection = selectionSIN(time);
  const std::vector<double> &curr = *currentData;

  unsigned int dim = 0;
  for (unsigned int i = 0; i < curr.size(); ++i)
    if (selection(i)) dim++;

  res.resize(dim);
  int cursor = 0;
  for (unsigned int i = 0; i < curr.size(); ++i)
    if (selection(i)) res(cursor++) = curr[i];

  sotDEBUGOUT(15);
  return res;
}

dynamicgraph::Matrix &sotReader::getNextMatrix(dynamicgraph::Matrix &res,
                                               const unsigned int time) {
  sotDEBUGIN(15);
  const dynamicgraph::Vector &vect = vectorSOUT(time);
  if (vect.size() < rows * cols) return res;

  res.resize(rows, cols);
  for (int i = 0; i < rows; ++i)
    for (int j = 0; j < cols; ++j) res(i, j) = vect(i * cols + j);

  sotDEBUGOUT(15);
  return res;
}
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void sotReader::display(std::ostream &os) const {
  os << CLASS_NAME << " " << name << endl;
}

std::ostream &operator<<(std::ostream &os, const sotReader &t) {
  t.display(os);
  return os;
}

/* --- Command line interface
 * ------------------------------------------------------ */
void sotReader::initCommands() {
  namespace dc = ::dynamicgraph::command;
  addCommand("clear", dc::makeCommandVoid0(*this, &sotReader::clear,
                                           "Clear the data loaded"));
  addCommand("rewind",
             dc::makeCommandVoid0(
                 *this, &sotReader::rewind,
                 "Reset the iterator to the beginning of the data set"));
  addCommand("load",
             dc::makeCommandVoid1(*this, &sotReader::load, "load file"));
  addCommand("resize", dc::makeCommandVoid2(*this, &sotReader::resize, " "));
}

void sotReader::resize(const int &row, const int &col) {
  rows = row;
  cols = col;
}
