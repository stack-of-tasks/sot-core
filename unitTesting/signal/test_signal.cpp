/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet VISTA / IRISA, 2003
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_categorie.cc
 * Project:   Traces
 * Author:    Nicolas Mansard
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
#include <iostream>
#include <MatrixAbstractLayer/boost.h>
using namespace std;
using namespace dynamicgraph;

namespace ml = maal::boost;

class DummyClass
{

public:
  ml::Vector& fun( ml::Vector& res,double j ) 
  { res.resize(3); res.fill(j); return res; }


};

ml::Vector data(6);
Signal<ml::Vector,double> sig("sigtest");
DummyClass dummy;

ml::Vector& fun( ml::Vector& res,double j ) { return res=data; }

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
