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

#include <sot/core/integrator.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/real-time-logger.h>
#include <dynamic-graph/signal-caster.h>
#include <dynamic-graph/signal.h>

namespace dynamicgraph {
namespace sot{
namespace internal {

Signal::Signal(std::string name) :
  ::dynamicgraph::Signal<Vector, int>(name)
{
}

/* ------------------------------------------------------------------------ */


void Signal::set(std::istringstream &stringValue) {
  (*this) = signal_io<Vector>::cast(stringValue);
}


void Signal::get(std::ostream &os) const {
  signal_io<Vector>::disp(this->accessCopy(), os);
}


void Signal::trace(std::ostream &os) const {
  try {
    signal_io<Vector>::trace(this->accessCopy(), os);
  } catch DG_RETHROW catch (...) {
    DG_THROW ExceptionSignal(ExceptionSignal::SET_IMPOSSIBLE,
                             "TRACE operation not possible with this signal. ",
                             "(bad cast while getting value from %s).",
                             SignalBase<int>::getName().c_str());
  }
}

void Signal::setConstant(const Vector &) {
  throw std::runtime_error("Not implemented.");
}


void Signal::setReference(const Vector *, Mutex *) {
  throw std::runtime_error("Not implemented.");
}


void Signal::setReferenceNonConstant(Vector *, Mutex *) {
  throw std::runtime_error("Not implemented.");
}


void Signal::setFunction(boost::function2<Vector &, Vector &, int> t,
                                  Mutex *mutexref) {
  signalType = ::dynamicgraph::Signal<Vector, int>::FUNCTION;
  Tfunction = t;
  providerMutex = mutexref;
  copyInit = false;
  setReady();
}


const Vector &Signal::accessCopy() const {
  return Tcopy1;
}


const Vector &Signal::access(const int &t) {
  if (NULL == providerMutex) {
    signalTime = t;
    Tfunction(Tcopy1, t);
    return Tcopy1;
  } else {
    try {
#ifdef HAVE_LIBBOOST_THREAD
      boost::try_mutex::scoped_try_lock lock(*providerMutex);
#endif
      signalTime = t;
      Tfunction(Tcopy1, t);
      return Tcopy1;
    } catch (const MutexError &) {
      return accessCopy();
    }
  }
}


Signal &Signal::operator=(const Vector &t) {
  throw std::runtime_error("Output signal cannot be assigned a value.");
  return *this;
}


std::ostream &Signal::display(std::ostream &os) const {
  os << "Sig:" << this->name << " (Type ";
  switch (this->signalType) {
    case Signal::CONSTANT:
      os << "Cst";
      break;
    case Signal::REFERENCE:
      os << "Ref";
      break;
    case Signal::REFERENCE_NON_CONST:
      os << "RefNonCst";
      break;
    case Signal::FUNCTION:
      os << "Fun";
      break;
  }
  return os << ")";
}

} // namespace internal
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Integrator, "Integrator");

const double Integrator::dt = 1e-6;

Integrator::Integrator(const std::string& name) :
  Entity(name),
  velocitySIN_(0x0, "Integrator(" + name + ")::input(vector)::velocity"),
  configurationSOUT_("Integrator(" + name + ")::output(vector)::configuration"),
  model_(0x0), configuration_(), lastComputationTime_(-1), recursivityLevel_(0)
{
  configurationSOUT_.setFunction(boost::bind(&Integrator::integrate, this, _1,
                                             _2));
  signalRegistration(velocitySIN_);
  signalRegistration(configurationSOUT_);
}

::pinocchio::Model* Integrator::getModel()
{
  return model_;
}

void Integrator::setModel(::pinocchio::Model* model)
{
  model_ = model;
  configuration_.resize(model->nq);
  ::pinocchio::neutral(*model_, configuration_);
}

void Integrator::setInitialConfig(const Vector& initConfig)
{
  configuration_ = initConfig;
}

Vector& Integrator::integrate(Vector& configuration, int time)
{
  ++recursivityLevel_;
  if (recursivityLevel_ == 2){
    configuration = configuration_;
    --recursivityLevel_;
    return configuration;
  }
  configuration.resize(configuration_.size());
  assert(model_);
  Vector velocity(velocitySIN_(time));
  // Run Synchronous commands and evaluate signals outside the main
  // connected component of the graph.
  try {
    periodicCallBefore_.run(time);
  } catch (const std::exception &e) {
    dgRTLOG() << "exception caught while running periodical commands (before): "
              << e.what() << std::endl;
  }
  if (lastComputationTime_ == -1 && !velocity.isZero(0)) {
    recursivityLevel_ = 0;
    std::ostringstream os;
    os << "Integrator entity expects zero velocity input for the first "
      "computation. Got instead " << velocity.transpose() << ".";
    throw std::runtime_error(os.str().c_str());
  }
  double delta_t = dt*(time - lastComputationTime_);
  ::pinocchio::integrate(*model_, configuration_, delta_t * velocity,
                         configuration);
  configuration_ = configuration;
  lastComputationTime_ = time;
  try {
    periodicCallAfter_.run(time);
  } catch (const std::exception &e) {
    dgRTLOG() << "exception caught while running periodical commands (after): "
              << e.what() << std::endl;
  }
  --recursivityLevel_;
  return configuration;
}

} // namespace sot
} // namespace dynamicgraph
