// BSD 2-Clause License

// Copyright (c) 2023, CNRS
// Author: Florent Lamiraux

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef SOT_DYNAMIC_PINOCCHIO_INTEGRATOR_HH
#define SOT_DYNAMIC_PINOCCHIO_INTEGRATOR_HH

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <sot/core/config.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include "sot/core/periodic-call.hh"

namespace dynamicgraph {
namespace sot {
namespace internal {
class Signal : public ::dynamicgraph::Signal<Vector, sigtime_t> {
 protected:
  enum SignalType { CONSTANT, REFERENCE, REFERENCE_NON_CONST, FUNCTION };
  static const SignalType SIGNAL_TYPE_DEFAULT = CONSTANT;

  const Vector *Treference;
  Vector *TreferenceNonConst;
  boost::function2<Vector &, Vector &, sigtime_t> Tfunction;

  bool keepReference;
  const static bool KEEP_REFERENCE_DEFAULT = false;

 public:
#ifdef HAVE_LIBBOOST_THREAD
  typedef boost::try_mutex Mutex;
  typedef boost::lock_error MutexError;
#else
  typedef size_type *Mutex;
  typedef size_type *MutexError;
#endif

 protected:
  Mutex *providerMutex;
  using SignalBase<sigtime_t>::signalTime;

 public:
  using SignalBase<sigtime_t>::setReady;

 public:
  /* --- Constructor/destrusctor --- */
  Signal(std::string name);
  virtual ~Signal() {}

  /* --- Generic In/Out function --- */
  virtual void get(std::ostream &value) const;
  virtual void set(std::istringstream &value);
  virtual void trace(std::ostream &os) const;

  /* --- Generic Set function --- */
  virtual void setConstant(const Vector &t);
  virtual void setReference(const Vector *t, Mutex *mutexref = NULL);
  virtual void setReferenceNonConstant(Vector *t, Mutex *mutexref = NULL);
  virtual void setFunction(boost::function2<Vector &, Vector &, sigtime_t> t,
                           Mutex *mutexref = NULL);

  /* --- Signal computation --- */
  virtual const Vector &access(const sigtime_t &t);
  virtual inline void recompute(const sigtime_t &t) { access(t); }
  virtual const Vector &accessCopy() const;

  virtual std::ostream &display(std::ostream &os) const;

  /* --- Operators --- */
  virtual inline const Vector &operator()(const sigtime_t &t) {
    return access(t);
  }
  virtual Signal &operator=(const Vector &t);
  inline operator const Vector &() const { return accessCopy(); }
  virtual void getClassName(std::string &aClassName) const {
    aClassName = typeid(this).name();
  }
};

}  // namespace internal
// Integrates a constant velocity for a given timestep
//
// Initial and final configurations as well as velocity follow pinocchio
// standard
// The timestep is the time elapsed since last computation of the output in
// microseconds.
class SOT_CORE_DLLEXPORT Integrator : public Entity {
 public:
  // Time corresponding to incrementing signal velocity by 1
  static const double dt;
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }
  Integrator(const std::string &name);

  // Get pointer to the model
  ::pinocchio::Model *getModel();
  // Set pointer to the model
  void setModel(::pinocchio::Model *model);
  // Set Initial configuration
  void setInitialConfig(const Vector &initConfig);

  PeriodicCall &periodicCallBefore() { return periodicCallBefore_; }
  PeriodicCall &periodicCallAfter() { return periodicCallAfter_; }

 private:
  PeriodicCall periodicCallBefore_;
  PeriodicCall periodicCallAfter_;

  Vector &integrate(Vector &configuration, sigtime_t time);
  // Signals
  SignalPtr<Vector, sigtime_t> velocitySIN_;
  internal::Signal configurationSOUT_;
  // Pointer to pinocchio model
  ::pinocchio::Model *model_;
  Vector configuration_;
  sigtime_t lastComputationTime_;
  sigtime_t recursivityLevel_;
};  // class Integrator

}  // namespace sot
}  // namespace dynamicgraph
#endif  // SOT_DYNAMIC_PINOCCHIO_INTEGRATOR_HH
