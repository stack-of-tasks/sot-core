// Copyright (c) 2017, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of sot-core.
// sot-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// sot-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-core. If not, see <http://www.gnu.org/licenses/>.

#include <sot/core/event.hh>

#include <dynamic-graph/factory.h>

namespace dynamicgraph {
  namespace sot {

    bool& Event::check (bool& ret, const int& time)
    {
      const bool& val = conditionSIN (time);
      ret = (val != lastVal_);
      bool trigger = onlyUp_ ? (!lastVal_ && val) : ret;
      if (ret) {
        lastVal_ = val;
        if (trigger) {
          for (Triggers_t::const_iterator _s = triggers.begin();
              _s != triggers.end(); ++_s)
            (*_s)->recompute (time);
        }
      }
      return ret;
    }

    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (Event, "Event");
  } // namespace sot
} // namespace dynamicgraph
