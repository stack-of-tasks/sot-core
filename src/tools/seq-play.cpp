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
#include <sot/core/debug.hh>
#include <sot/core/seq-play.hh>
using namespace std;

#include <fstream>
#include <sstream>

#include <dynamic-graph/factory.h>
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SeqPlay, "SeqPlay");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

SeqPlay::SeqPlay(const std::string &n)
    : Entity(n), stateList(), currPos(stateList.begin()), currRank(0),
      init(false), time(0),
      refresherSINTERN("SeqPlay(" + n + ")::intern(dummy)::refresher"),
      positionSOUT(boost::bind(&SeqPlay::getNextPosition, this, _1, _2),
                   refresherSINTERN,
                   "SeqPlay(" + n + ")::output(vector)::position") {
  signalRegistration(positionSOUT);
  refresherSINTERN.setDependencyType(TimeDependency<int>::ALWAYS_READY);
}

/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */
dynamicgraph::Vector &SeqPlay::getNextPosition(dynamicgraph::Vector &pos,
                                               const int & /*time*/) {
  sotDEBUGIN(15);
  if (!init) {
    if (stateList.empty())
      return pos;
    currPos = stateList.begin();
    init = true;
    currRank = 0;
  }

  {
    const dynamicgraph::Vector &posCur = *currPos;
    pos = posCur;

    currPos++;
    if (currPos == stateList.end())
      currPos--;
    else
      currRank++;
  }

  sotDEBUGOUT(15);
  return pos;
}

/* --- LIST -------------------------------------------------------------- */
/* --- LIST -------------------------------------------------------------- */
/* --- LIST -------------------------------------------------------------- */
void SeqPlay::loadFile(const std::string &filename) {
  sotDEBUGIN(15);

  sotDEBUG(25) << " Load " << filename << endl;
  std::ifstream file(filename.c_str());
  const unsigned int SIZE = 1024;
  char buffer[SIZE];

  dynamicgraph::Vector res(1);
  unsigned int ressize = 1;
  double time;

  while (file.good()) {
    file.getline(buffer, SIZE);
    if (file.gcount() < 5)
      break;

    sotDEBUG(25) << buffer << endl;
    std::istringstream iss(buffer);

    iss >> time;
    unsigned int i;

    for (i = 0; iss.good(); ++i) {
      if (i == ressize) {
        ressize *= 2;
        res.resize(ressize, false);
      }
      iss >> res(i);
      sotDEBUG(35) << i << ": " << res(i) << endl;
    }
    ressize = i - 1;
    res.resize(ressize, false);
    stateList.push_back(res);
    sotDEBUG(15) << time << ": " << res << endl;
  }

  sotDEBUGOUT(15);
}

/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

void SeqPlay::display(std::ostream &os) const { os << name << endl; }
