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

template <class Res = double>
class DummyClass {
 public:
  DummyClass(void) : res(), appel(0), timedata(0) {}

  Res &fun(Res &res, int t) {
    appel++;
    timedata = t;

    cout << "Inside " << endl;
    for (list<SignalTimeDependent<double, int> *>::iterator it =
             inputsig.begin();
         it != inputsig.end(); ++it) {
      cout << *(*it) << endl;
      (*it)->access(timedata);
    }
    for (list<SignalTimeDependent<dynamicgraph::Vector, int> *>::iterator it =
             inputsigV.begin();
         it != inputsigV.end(); ++it) {
      cout << *(*it) << endl;
      (*it)->access(timedata);
    }

    return res = (*this)();
  }

  list<SignalTimeDependent<double, int> *> inputsig;
  list<SignalTimeDependent<dynamicgraph::Vector, int> *> inputsigV;

  void add(SignalTimeDependent<double, int> &sig) { inputsig.push_back(&sig); }
  void add(SignalTimeDependent<dynamicgraph::Vector, int> &sig) {
    inputsigV.push_back(&sig);
  }

  Res operator()(void);

  Res res;
  int appel;
  int timedata;
};

template <class Res>
Res DummyClass<Res>::operator()(void) {
  return this->res;
}

template <>
double DummyClass<double>::operator()(void) {
  res = appel * timedata;
  return res;
}
template <>
dynamicgraph::Vector DummyClass<dynamicgraph::Vector>::operator()(void) {
  res.resize(3);
  res.fill(appel * timedata);
  return res;
}

// void dispArray( const SignalArray<int> &ar )
// {
//   for( unsigned int i=0;i<ar.rank;++i ) cout<<*ar.array[i]<<endl;
// }

#include <vector>
int main(void) {
  DummyClass<double> pro1, pro3, pro5;
  DummyClass<dynamicgraph::Vector> pro2, pro4, pro6;

  SignalTimeDependent<double, int> sig5("Sig5");
  SignalTimeDependent<dynamicgraph::Vector, int> sig6("Sig6");

  SignalTimeDependent<dynamicgraph::Vector, int> sig4(sig5, "Sig4");
  SignalTimeDependent<dynamicgraph::Vector, int> sig2(
      sig4 << sig4 << sig4 << sig6, "Sig2");
  SignalTimeDependent<double, int> sig3(sig2 << sig5 << sig6, "Sig3");
  SignalTimeDependent<double, int> sig1(
      boost::bind(&DummyClass<double>::fun, pro1, _1, _2), sig2 << sig3,
      "Sig1");

  //    cout << "--- Test Array ------ "<<endl;
  //    SignalArray<int> tarr(12);
  //    tarr<<sig3<<sig2;//+sig2+sig3;
  //    dispArray(sig4<<sig2<<sig3);
  //    dispArray(tarr);

  // sig1.set( &DummyClass<double>::fun,pro1 );
  sig2.setFunction(
      boost::bind(&DummyClass<dynamicgraph::Vector>::fun, pro2, _1, _2));
  sig3.setFunction(boost::bind(&DummyClass<double>::fun, pro3, _1, _2));
  sig4.setFunction(
      boost::bind(&DummyClass<dynamicgraph::Vector>::fun, pro4, _1, _2));
  sig5.setFunction(boost::bind(&DummyClass<double>::fun, pro5, _1, _2));
  sig6.setFunction(
      boost::bind(&DummyClass<dynamicgraph::Vector>::fun, pro6, _1, _2));

  //    sig1.addDependency(sig2);
  //     sig1.addDependency(sig3);
  //      sig2.addDependency(sig4);
  //      sig2.addDependency(sig4);
  //     sig2.addDependency(sig4);
  //      sig3.addDependency(sig2);
  //      sig4.addDependency(sig5);
  //      sig2.addDependency(sig6);
  //      sig3.addDependency(sig5);
  //      sig3.addDependency(sig6);

  pro1.add(sig2);
  pro1.add(sig3);
  pro2.add(sig4);
  pro2.add(sig4);
  pro2.add(sig4);
  pro3.add(sig2);
  pro4.add(sig5);
  pro2.add(sig6);
  pro3.add(sig5);
  pro3.add(sig6);

  // sig5.setDependencyType(TimeDependency<int>::ALWAYS_READY);
  // sig6.setDependencyType(TimeDependency<int>::BOOL_DEPENDENT);

  sig6.setReady();

  sig1.displayDependencies(cout) << endl;

  cout << "Needs update?" << endl << sig1.needUpdate(2) << endl;
  sig1.access(2);
  sig1.displayDependencies(cout) << endl;
  sig2.access(4);
  sig1.displayDependencies(cout) << endl;
  sig1.access(4);
  sig1.displayDependencies(cout) << endl;
  sig1.needUpdate(6);
  sig1.needUpdate(6);

  return 0;
}
