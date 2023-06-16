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

#include <iostream>
#include <sot/core/segment.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

namespace dynamicgraph {
namespace sot {

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Segment, "Segment");

Segment::Segment(const std::string& name)
    : Entity(name),
      inputSIN_(0x0, "Segment(" + name + ")::input(vector)::in"),
      outputSOUT_(boost::bind(&Segment::compute, this, _1, _2), inputSIN_,
                  "Segment(" + name + ")::out"),
      range_(std::make_pair(0, 0)) {
  using dynamicgraph::command::makeCommandVoid2;
  signalRegistration(inputSIN_);
  signalRegistration(outputSOUT_);
  std::string docstring(
      "Set range of input vector to be provided as output\n"
      "  - input: first index, length\n");
  addCommand("setRange",
             makeCommandVoid2(*this, &Segment::setRange, docstring));
}

Vector& Segment::compute(Vector& output, sigtime_t time) {
  const Vector& input(inputSIN_(time));
  output = input.segment(std::get<0>(range_), std::get<1>(range_));
  return output;
}
}  // namespace sot
}  // namespace dynamicgraph
