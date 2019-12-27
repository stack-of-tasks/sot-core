// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)

#include <sot/core/event.hh>

#include <dynamic-graph/factory.h>

namespace dynamicgraph {
namespace sot {

bool &Event::check(bool &ret, const int &time) {
  const bool &val = conditionSIN(time);
  ret = (val != lastVal_);
  bool trigger = onlyUp_ ? (!lastVal_ && val) : ret;
  if (ret) {
    lastVal_ = val;
    if (trigger) {
      for (Triggers_t::const_iterator _s = triggers.begin();
           _s != triggers.end(); ++_s)
        (*_s)->recompute(time);
    }
  }
  return ret;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Event, "Event");
} // namespace sot
} // namespace dynamicgraph
