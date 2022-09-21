// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)

#include <dynamic-graph/factory.h>

#include <sot/core/event.hh>

namespace dynamicgraph {
namespace sot {

bool &Event::check(bool &ret, const int &time) {
  const bool &val = conditionSIN(time);
  ret = (val != lastVal_);
  bool up = (!lastVal_ && val);
  if (up) {
    timeSinceUp_ = 0;
  } else if (val) {
    ++timeSinceUp_;
  }
  // If repetition is activated, trigger again after given number of iterations
  bool trigger = up || (repeatAfterNIterations_ > 0 &&
                        timeSinceUp_ >= repeatAfterNIterations_);
  if (ret) {
    lastVal_ = val;
  }
  if (trigger) {
    for (Triggers_t::const_iterator _s = triggers.begin(); _s != triggers.end();
         ++_s)
      (*_s)->recompute(time);
    timeSinceUp_ = 0;
  }
  return ret;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Event, "Event");
}  // namespace sot
}  // namespace dynamicgraph
