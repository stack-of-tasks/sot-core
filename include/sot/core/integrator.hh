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
#include <dynamic-graph/signal-ptr.t.cpp>
#include <dynamic-graph/signal-time-dependent.h>

namespace dynamicgraph {
namespace sot{

// Integrates a constant velocity for a given timestep
//
// Initial and final configurations as well as velocity follow pinocchio
// standard
// The timestep is the time elapsed since last computation of the output in
// microseconds.
class SOT_CORE_DLLEXPORT Integrator : public Entity
{
public:
  // Time corresponding to incrementing signal velocity by 1
  static const double dt;
  static const std::string CLASS_NAME;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }
  Integrator(const std::string& name);
      
  // Get pointer to the model
  ::pinocchio::Model* getModel();
  // Set pointer to the model
  void setModel(::pinocchio::Model* model);
  // Set Initial configuration
  void setInitialConfig(const Vector& initConfig);
  
private:
  Vector& integrate(Vector& configuration, int time);
  // Signals
  SignalPtr<Vector, int> velocitySIN_;
  SignalTimeDependent<Vector, int> configurationSOUT_;
  // Pointer to pinocchio model
  ::pinocchio::Model* model_;
  Vector configuration_;
  int lastComputationTime_;
}; // class Integrator

} // namespace sot
} // namespace dynamicgraph
#endif //SOT_DYNAMIC_PINOCCHIO_INTEGRATOR_HH
