/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-base.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/tracer.h>

#include <iostream>
#include <sot/core/debug.hh>

using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

double &f(double &res, const int & /*t*/) {
  cout << "SIGM!" << endl;
  return res;
}

int main() {
  Signal<Vector, int> sig1("sig1");
  Vector v1(2);
  v1.fill(1.1);
  sig1 = v1;

  Signal<Vector, int> sig2("sig2");
  Vector v2(4);
  v2.fill(2.);
  sig2 = v2;

  Signal<Vector, int> sig3("sig3");
  Vector v3(6);
  v3.fill(3.);
  sig3 = v3;

  SignalTimeDependent<double, int> sigM(f, sotNOSIGNAL, "sigM");
  sigM.access(0);

  Tracer *tracer = new Tracer("trace");
  tracer->addSignalToTrace(sig1);
  tracer->openFiles("/tmp/sot-core", "tr_", ".dat");

  tracer->addSignalToTrace(sig2);

  return 0;
}
