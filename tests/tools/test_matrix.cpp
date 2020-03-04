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

#include <dynamic-graph/linear-algebra.h>
#include <sot/core/debug.hh>
#include <sot/core/feature-abstract.hh>

#include <iostream>
using namespace std;

#include <sot/core/sot.hh>

#ifndef WIN32
#include <sys/time.h>
#else /*WIN32*/
#include <sot/core/utils-windows.hh>
#endif /*WIN32*/

#define sotCHRONO1                                                             \
  gettimeofday(&t1, NULL);                                                     \
  dt = ((t1.tv_sec - t0.tv_sec) * 1000. +                                      \
        (t1.tv_usec - t0.tv_usec + 0.) / 1000.);                               \
  cout << "dt: " << dt

int main(int, char **) {
  sotDEBUGIN(15);

  struct timeval t0, t1;
  double dt;

  dynamicgraph::Matrix P(40, 40);
  dynamicgraph::Matrix J(6, 40);
  dynamicgraph::Matrix JK(6, 40);
  for (int i = 0; i < 40; ++i)
    for (int j = 0; j < 40; ++j)
      P(i, j) = (rand() + 1.) / RAND_MAX;
  for (int i = 0; i < J.rows(); ++i)
    for (int j = 0; j < J.cols(); ++j)
      J(i, j) = (rand() + 1.) / RAND_MAX;

  int nbIter = 100000;
  dt = 0;
  gettimeofday(&t0, NULL);
  for (int iter = 0; iter < nbIter; ++iter) {
    gettimeofday(&t0, NULL);
    // J.multiply(P,JK);
    // prod(J.matrix,P.matrix,JK.matrix);
    JK = J * P;
    gettimeofday(&t1, NULL);
    dt += ((double)(t1.tv_sec - t0.tv_sec) +
           (double)(t1.tv_usec - t0.tv_usec) / 1000. / 1000.);
  }
  // sotCHRONO1 <<endl;
  cout << dt / nbIter << endl;

  sotDEBUGOUT(15);
  return 0;
}
