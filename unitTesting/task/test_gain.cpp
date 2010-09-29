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

#include <dynamic-graph/signal.h>
#include <sot-core/gain-adaptive.h>
#include <iostream>
using namespace std;
using namespace sot;
using namespace dynamicgraph;

namespace ml = maal::boost;

class DummyClass
{
public:
  ml::Vector err;

  ml::Vector& getError( ml::Vector& res,int t )
  {
    cout << "Dummy::getError ["<< t<< "] "<<endl;
    return res=err;
  }

};

DummyClass dummy;



/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

int main( void )
{
  dummy.err.resize(3);
  dummy.err.fill(3);

  GainAdaptive * gain = new GainAdaptive("gain",4,1,5);

  Signal<ml::Vector,int> errSig("test");
  errSig.setFunction( boost::bind(&DummyClass::getError,dummy,_1,_2) );

  gain->errorSIN.plug( &errSig );
  cout <<"Appel of errSig and display of the result." << endl;
  cout << errSig(0) <<endl;

//   double res;
//   cout <<"Compute gain from Gain object."<< endl;
//   gain->computeGain( res,0 );

  Signal<double,int> &gainSig = gain->gainSOUT;

  cout <<"Compute gain from Gain Signal and display."<< endl;
  cout << gainSig(0) << endl;

//   dg::SignalPtr<ml::Vector,int> sigPtr( &errSig );
//   cout << errSig(0) <<endl;
//   cout << (*sigPtr).access(0) <<endl;

  delete gain;

  return 0;
}
