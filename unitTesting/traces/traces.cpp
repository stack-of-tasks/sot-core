/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_traces.cpp
 * Project:   sot
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <iostream>

#include <sot-core/debug.h>
#include <dynamic-graph/signal-base.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/tracer.h>
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

using namespace std;
using namespace dynamicgraph;
using namespace sot;

double& f( double& res,const int& t ) { cout << "SIGM!"<<endl; return res; }

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
 
