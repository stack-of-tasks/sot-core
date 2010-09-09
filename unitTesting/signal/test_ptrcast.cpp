/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet VISTA / IRISA, 2003
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_categorie.cc
 * Project:   Traces
 * Author:    François Bleibel
 *
 * Version control
 * ===============
 *
 *  $Id: test_boost.cpp,v 1.1.1.1 2006-07-03 05:17:37 nmansard Exp $
 *
 * Description
 * ============
 *
 * Test la classe CategorieTrace.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <dynamic-graph/all-signals.h>
#include <sot-core/matrix-rotation.h>
#include <iostream>
#include <MatrixAbstractLayer/boost.h>
using namespace std;
using namespace dynamicgraph;
using namespace sot;

namespace ml = maal::boost;

Signal<ml::Matrix,int> base("base");
Signal<ml::Matrix,int> sig("matrix");
SignalPtr<ml::Matrix,int> sigptr(&base);

Signal<MatrixRotation,int> sigMR("matrixRot");

int main( void )
{
  sigptr.plug(&sig);
  cout << "Correctly plugged matrix" << endl;
  sigptr.plug(&sigMR);
  cout << "Correctly plugged matrix rotation" << endl;

  return 0;
}
