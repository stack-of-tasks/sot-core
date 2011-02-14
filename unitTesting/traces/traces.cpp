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

#include <iostream>

#include <sot-core/debug.h>
#include <dynamic-graph/signal-base.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/tracer.h>
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

double& f( double& res,const int& /*t*/ ) { cout << "SIGM!"<<endl; return res; }

int main()
{
  Signal<ml::Vector,int> sig1( "sig1" );
  ml::Vector v1(2); v1.fill(1.1); sig1 = v1;

  Signal<ml::Vector,int> sig2( "sig2" );
  ml::Vector v2(4); v2.fill(2.); sig2 = v2;

  Signal<ml::Vector,int> sig3( "sig3" );
  ml::Vector v3(6); v3.fill(3.); sig3 = v3;

  SignalTimeDependent<double,int> sigM( f,sotNOSIGNAL,"sigM" );
  sigM.access(0);
  
  Tracer tracer( "trace" );
  tracer.addSignalToTrace( sig1 );
  tracer.openFiles( "/tmp/sot-core","tr_",".dat" );
  
  tracer.addSignalToTrace( sig2 );

  return 0;
}
 
