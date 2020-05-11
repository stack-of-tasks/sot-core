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
using namespace std;
using namespace dynamicgraph;

class DummyClass {

public:
  dynamicgraph::Vector &fun(dynamicgraph::Vector &res, double j) {
    res.resize(3);
    res.fill(j);
    return res;
  }
};

dynamicgraph::Vector data(6);
Signal<dynamicgraph::Vector, double> sig("sigtest");
DummyClass dummy;

dynamicgraph::Vector &fun(dynamicgraph::Vector &res, double /*j*/) {
  return res = data;
}

int main(void) {
  data.fill(1);
  cout << "data: " << data << endl;

  sig.setConstant(data);
  cout << "Constant: " << sig.access(1.) << endl;
  data *= 2;
  cout << "Constant: " << sig(1.) << endl;

  sig.setReference(&data);
  cout << "Reference: " << sig(1.) << endl;
  data *= 2;
  cout << "Reference: " << sig(1.) << endl;

  sig.setFunction(&fun);
  cout << "Function: " << sig(1.) << endl;
  data *= 2;
  cout << "Function: " << sig(1.) << endl;

  // boost::function2<int,int,double> onClick = (&DummyClass::fun, &dummy,
  // _1,_2)   ; boost::function<> onClick = boost::bind(&DummyClass::fun,
  // &dummy);
  sig.setFunction(boost::bind(&DummyClass::fun, &dummy, _1, _2));
  cout << "Function: " << sig(1.5) << endl;
  data *= 2;
  cout << "Function: " << sig(1.34) << endl;

  //   sig.setFunction(&DummyClass::fun, dummy);
  //   cout << "Function: " << sig(1.5) <<endl;
  //   data*=2;
  //   cout << "Function: " << sig(12.34) <<endl;

  return 0;
}
