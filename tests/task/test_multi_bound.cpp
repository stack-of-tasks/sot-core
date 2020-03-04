/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <iostream>
#include <sot/core/debug.hh>
#include <sot/core/multi-bound.hh>
#include <sstream>

using namespace std;
using namespace dynamicgraph::sot;

int main(void) {
  DebugTrace::openFile();

  MultiBound mbs(1.2), mbdi(3.4, MultiBound::BOUND_INF),
      mbds(5.6, MultiBound::BOUND_SUP), mbdb(-7.8, 9.10);
  cout << "mbs =" << mbs << std::endl;
  cout << "mbdi=" << mbdi << std::endl;
  cout << "mbds=" << mbds << std::endl;
  cout << "mbdb=" << mbdb << std::endl;

  {
    ostringstream oss;
    istringstream iss;
    oss.str("");
    oss << mbs;
    iss.str(oss.str());
    iss.seekg(0);
    //{char strbuf[256]; iss.getline(strbuf,256); cout <<
    //"#"<<strbuf<<"#"<<std::endl;}
    iss >> mbs;
    cout << oss.str() << "=> mbs =" << mbs << std::endl;
  }

  {
    ostringstream oss;
    istringstream iss;
    oss.str("");
    oss << mbdi;
    iss.str(oss.str());
    iss.seekg(0);
    iss >> mbdi;
    cout << oss.str() << "=> mbdi =" << mbdi << std::endl;
  }
  {
    ostringstream oss;
    istringstream iss;
    oss.str("");
    oss << mbds;
    iss.str(oss.str());
    iss.seekg(0);
    iss >> mbds;
    cout << oss.str() << "=> mbds =" << mbds << std::endl;
  }
  {
    ostringstream oss;
    istringstream iss;
    oss.str("");
    oss << mbdb;
    iss.seekg(0);
    iss.str(oss.str());
    iss >> mbdb;
    cout << oss.str() << "=> mbdb =" << mbdb << std::endl;
  }

  ostringstream oss;
  istringstream iss;
  oss << "[4](" << mbs << "," << mbdi << "," << mbds << "," << mbdb << ")"
      << std::endl;
  iss.str(oss.str());
  VectorMultiBound vmb;
  iss >> vmb;
  cout << "vmb4 = " << vmb << std::endl;

  return 0;
}
