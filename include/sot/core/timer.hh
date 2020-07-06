/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef __SOT_TIMER_HH
#define __SOT_TIMER_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Classes standards. */
#include <list> /* Classe std::list   */
#ifndef WIN32
#include <sys/time.h>
#else /*WIN32*/
// When including Winsock2.h, the MAL must be included first
#include <Winsock2.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/utils-windows.hh>
#endif /*WIN32*/

/* SOT */
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/entity.h>
#include <sot/core/debug.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined(WIN32)
#if defined(timer_EXPORTS)
#define Timer_EXPORT __declspec(dllexport)
#else
#define Timer_EXPORT __declspec(dllimport)
#endif
#else
#define Timer_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

template <class T> class Timer_EXPORT Timer : public dynamicgraph::Entity {
public:
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

protected:
  struct timeval t0, t1;
  clock_t c0, c1;
  double dt;

public:
  /* --- CONSTRUCTION --- */
  Timer(const std::string &name);

public: /* --- DISPLAY --- */
  virtual void display(std::ostream &os) const;
  Timer_EXPORT friend std::ostream &operator<<(std::ostream &os,
                                               const Timer<T> &timer) {
    timer.display(os);
    return os;
  }

public: /* --- SIGNALS --- */
  dynamicgraph::SignalPtr<T, int> sigSIN;
  dynamicgraph::SignalTimeDependent<T, int> sigSOUT;
  dynamicgraph::SignalTimeDependent<T, int> sigClockSOUT;
  dynamicgraph::Signal<double, int> timerSOUT;

protected: /* --- SIGNAL FUNCTIONS --- */
  void plug(dynamicgraph::Signal<T, int> &sig) {
    sigSIN = &sig;
    dt = 0.;
  }

  template <bool UseClock> T &compute(T &t, const int &time) {
    sotDEBUGIN(15);
    if (UseClock) {
      c0 = clock();
      sotDEBUG(15) << "t0: " << c0 << std::endl;
    } else {
      gettimeofday(&t0, NULL);
      sotDEBUG(15) << "t0: " << t0.tv_sec << " - " << t0.tv_usec << std::endl;
    }

    t = sigSIN(time);

    if (UseClock) {
      c1 = clock();
      sotDEBUG(15) << "t1: " << c0 << std::endl;
      dt = ((double)(c1 - c0) * 1000) / CLOCKS_PER_SEC;
    } else {
      gettimeofday(&t1, NULL);
      dt = ((static_cast<double>(t1.tv_sec) - static_cast<double>(t0.tv_sec)) *
                1000. +
            (static_cast<double>(t1.tv_usec) - static_cast<double>(t0.tv_usec) +
             0.) /
                1000.);
      sotDEBUG(15) << "t1: " << t1.tv_sec << " - " << t1.tv_usec << std::endl;
    }

    timerSOUT = dt;
    timerSOUT.setTime(time);

    sotDEBUGOUT(15);
    return t;
  }

  double &getDt(double &res, const int & /*time*/) {
    res = dt;
    return res;
  }
};

void cmdChrono(const std::string &cmd, std::istringstream &args,
               std::ostream &os);

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* --- CONSTRUCTION ---------------------------------------------------- */
template <class T>
Timer<T>::Timer(const std::string &name)
    : Entity(name), dt(0.), sigSIN(NULL, "Timer(" + name + ")::input(T)::sin"),
      sigSOUT(boost::bind(&Timer::compute<false>, this, _1, _2), sigSIN,
              "Timer(" + name + ")::output(T)::sout"),
      sigClockSOUT(boost::bind(&Timer::compute<true>, this, _1, _2), sigSIN,
                   "Timer(" + name + ")::output(T)::clockSout"),
      timerSOUT("Timer(" + name + ")::output(double)::timer") {
  sotDEBUGIN(15);
  timerSOUT.setFunction(boost::bind(&Timer::getDt, this, _1, _2));

  signalRegistration(sigSIN << sigSOUT << sigClockSOUT << timerSOUT);
  sotDEBUGOUT(15);
}

/* --- DISPLAY --------------------------------------------------------- */
template <class T> void Timer<T>::display(std::ostream &os) const {
  os << "Timer <" << sigSIN << "> : " << dt << "ms." << std::endl;
}

#endif /* #ifndef __SOT_SOT_HH */
