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
#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>
#include <sot/core/matrix-geometry.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;

template <class Res = double>
class DummyClass {
 public:
  DummyClass(void) : res(), appel(0), timedata(0) {}

  Res &fun(Res &res, int t) {
    appel++;
    timedata = t;

    sotDEBUG(5) << "Inside " << typeid(Res).name() << endl;
    for (list<SignalTimeDependent<double, sigtime_t> *>::iterator it =
             inputsig.begin();
         it != inputsig.end(); ++it) {
      sotDEBUG(5) << *(*it) << endl;
      (*it)->access(timedata);
    }
    for (list<SignalTimeDependent<dynamicgraph::Vector, sigtime_t> *>::iterator
             it = inputsigV.begin();
         it != inputsigV.end(); ++it) {
      sotDEBUG(5) << *(*it) << endl;
      (*it)->access(timedata);
    }

    return res = (*this)();
  }

  list<SignalTimeDependent<double, sigtime_t> *> inputsig;
  list<SignalTimeDependent<dynamicgraph::Vector, sigtime_t> *> inputsigV;

  void add(SignalTimeDependent<double, sigtime_t> &sig) {
    inputsig.push_back(&sig);
  }
  void add(SignalTimeDependent<dynamicgraph::Vector, sigtime_t> &sig) {
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
template <>
VectorUTheta DummyClass<VectorUTheta>::operator()(void) {
  res.angle() = 0.26;
  res.axis() = Eigen::Vector3d::UnitX();
  return res;
}

void funtest(dynamicgraph::Vector & /*v*/) {}

#include <vector>
int main(void) {
  DummyClass<VectorUTheta> pro3;
  SignalTimeDependent<VectorUTheta, sigtime_t> sig3(sotNOSIGNAL, "Sig3");
  SignalPtr<Eigen::AngleAxisd, sigtime_t> sigTo3(NULL, "SigTo3");
  dynamicgraph::Vector v;
  VectorUTheta v3;
  funtest(v);

  sig3.setFunction(boost::bind(&DummyClass<VectorUTheta>::fun, pro3, _1, _2));
  try {
    sigTo3.plug(&sig3);
  } catch (sot::ExceptionAbstract &e) {
    cout << "Plugin error " << e << endl;
    exit(1);
  }
  sig3.access(1);
  sig3.setReady();
  sigTo3.access(2);
  cout << sigTo3.access(2);

  return 0;
}
