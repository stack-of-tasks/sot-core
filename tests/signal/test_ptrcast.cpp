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
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/linear-algebra.h>

#include <iostream>
#include <sot/core/matrix-geometry.hh>
using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

Signal<dynamicgraph::Matrix, sigtime_t> base("base");
Signal<dynamicgraph::Matrix, sigtime_t> sig("matrix");
SignalPtr<dynamicgraph::Matrix, sigtime_t> sigptr(&base);

Signal<MatrixRotation, sigtime_t> sigMR("matrixRot");

int main(void) {
  sigptr.plug(&sig);
  cout << "Correctly plugged matrix" << endl;
  //  sigptr.plug(&sigMR);
  //  cout << "Correctly plugged matrix rotation" << endl;

  return 0;
}
