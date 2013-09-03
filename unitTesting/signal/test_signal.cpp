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
#include <iostream>

using namespace std;
using namespace dynamicgraph;

class DummyClass
{

public:
  Eigen::VectorXd& fun( Eigen::VectorXd& res,double j )
  { res.resize(3); res.fill(j); return res; }


};

Eigen::VectorXd data(6);
Signal<Eigen::VectorXd,double> sig("sigtest");
DummyClass dummy;

Eigen::VectorXd& fun( Eigen::VectorXd& res,double /*j*/ ) { return res=data; }

int main( void )
{
  data.fill(1);
  cout << "data: " << data <<endl;

  sig.setConstant( data );
  cout << "Constant: " << sig.access(1.)  <<endl;
  data*=2;
  cout << "Constant: " << sig(1.) <<endl;

  sig.setReference( &data );
  cout << "Reference: " << sig(1.) <<endl;
  data*=2;
  cout << "Reference: " << sig(1.) <<endl;

  sig.setFunction( &fun );
  cout << "Function: " << sig(1.) <<endl;
  data*=2;
  cout << "Function: " << sig(1.) <<endl;


  //boost::function2<int,int,double> onClick = (&DummyClass::fun, &dummy, _1,_2)   ;
  //boost::function<> onClick = boost::bind(&DummyClass::fun, &dummy);
  sig.setFunction( boost::bind(&DummyClass::fun, &dummy, _1,_2) );
  cout << "Function: " << sig(1.5) <<endl;
  data*=2;
  cout << "Function: " << sig(1.34) <<endl;


//   sig.setFunction(&DummyClass::fun, dummy);
//   cout << "Function: " << sig(1.5) <<endl;
//   data*=2;
//   cout << "Function: " << sig(12.34) <<endl;


  return 0;
}
