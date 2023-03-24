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

namespace dynamicgraph {
namespace sot{

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Integrator, "Integrator");

const double Integrator::dt = 1e-6;

Integrator::Integrator(const std::string& name) :
  Entity(name),
  velocitySIN_(0x0, "Integrator(" + name + ")::input(vector)::velocity"),
  configurationSOUT_(boost::bind(&Integrator::integrate, this, _1, _2),
                     velocitySIN_, "Integrator(" + name +
                     ")::output(vector)::configuration"), model_(0x0),
  configuration_(), lastComputationTime_(-1)
{
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
  assert(model_);
  Vector velocity(velocitySIN_(time));
  if (lastComputationTime_ == -1 && !velocity.isZero(0)) {
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
  return configuration;
}

} // namespace sot
} // namespace dynamicgraph
