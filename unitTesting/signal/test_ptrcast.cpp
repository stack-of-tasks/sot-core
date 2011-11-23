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
#include <dynamic-graph/all-signals.h>
#include <sot/core/matrix-rotation.hh>
#include <iostream>
#include <jrl/mal/malv2.hh>
using namespace std;
using namespace dynamicgraph;
using namespace dynamicgraph::sot;

DECLARE_MAL_NAMESPACE(ml);

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
