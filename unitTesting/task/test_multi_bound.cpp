/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <sot/core/multi-bound.hh>
#include <iostream>
#include <sstream>
#include <sot/core/debug.hh>

using namespace std;
using namespace dynamicgraph::sot;

int main( void )
{
   DebugTrace::openFile();

  MultiBound mbs(1.2), mbdi(3.4,MultiBound::BOUND_INF)
    ,mbds(5.6, MultiBound::BOUND_SUP),mbdb(-7.8,9.10);
  cout << "mbs =" << mbs << std::endl;
  cout << "mbdi=" << mbdi << std::endl;
  cout << "mbds=" << mbds << std::endl;
  cout << "mbdb=" << mbdb << std::endl;

  {
    ostringstream oss; istringstream iss;
    oss.str(""); oss << mbs; iss.str(oss.str()); iss.seekg(0);
    //{char strbuf[256]; iss.getline(strbuf,256); cout << "#"<<strbuf<<"#"<<std::endl;}
    iss >> mbs;
    cout << oss.str() << "=> mbs =" << mbs << std::endl;
  }

  {
    ostringstream oss; istringstream iss;
    oss.str(""); oss << mbdi; iss.str(oss.str()); iss.seekg(0);
    iss >> mbdi;
    cout << oss.str() << "=> mbdi =" << mbdi << std::endl;
  }
  {
    ostringstream oss; istringstream iss;
    oss.str(""); oss << mbds; iss.str(oss.str()); iss.seekg(0);
    iss >> mbds;
    cout << oss.str() << "=> mbds =" << mbds << std::endl;
  }
  {
    ostringstream oss; istringstream iss;
    oss.str(""); oss << mbdb; iss.seekg(0); iss.str(oss.str());
    iss >> mbdb;
    cout << oss.str() << "=> mbdb =" << mbdb << std::endl;
  }

  ostringstream oss; istringstream iss;
  oss << "[4](" << mbs << "," << mbdi << "," << mbds << "," << mbdb << ")" << std::endl;
  iss.str(oss.str());
  VectorMultiBound vmb; iss >> vmb;
  cout << "vmb4 = " << vmb << std::endl;

  return 0;
}
