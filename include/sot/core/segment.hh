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

#ifndef SOT_CORE_SEGMENT_HH
#define SOT_CORE_SEGMENT_HH

#include <sot/core/config.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.t.cpp>
#include <dynamic-graph/signal-time-dependent.h>

namespace dynamicgraph {
namespace sot {
class SOT_CORE_DLLAPI Segment : public Entity {
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName() const { return CLASS_NAME; }
  // Set the range of input vector that is provided as output.
  void setRange(const size_type& i0, const size_type& length) {
    range_ = std::make_pair(i0, length);
  }

  Segment(const std::string& name);

 private:
  Vector& compute(Vector& output, sigtime_t time);
  SignalPtr<Vector, sigtime_t> inputSIN_;
  SignalTimeDependent<Vector, sigtime_t> outputSOUT_;
  // Range of the input vector that is provided as output
  std::pair<size_type, size_type> range_;
};  // class Segment
}  // namespace sot
}  // namespace dynamicgraph
#endif  // SOT_CORE_SEGMENT_HH
