// Copyright (c) 2018, Joseph Mirabel
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

#ifndef __SOT_EVENT_H__
# define __SOT_EVENT_H__

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>

#include <sot/core/config.hh>

namespace dynamicgraph {
  namespace sot {
    /// Event
    class SOT_CORE_DLLAPI Event : public dynamicgraph::Entity
    {
      DYNAMIC_GRAPH_ENTITY_DECL();

      Event (const std::string& name) :
        Entity (name),
        checkSOUT ("Event("+name+")::output(bool)::check"),
        conditionSIN(NULL,"Event("+name+")::input(bool)::condition"),
        lastVal_ (2) // lastVal_ should be different true and false.
      {
        checkSOUT.setFunction
          (boost::bind (&Event::check, this, _1, _2));
        signalRegistration (conditionSIN);
        signalRegistration (checkSOUT);

        using command::makeCommandVoid1;
        std::string docstring =
          "\n"
          "    Add a signal\n";
        addCommand ("addSignal", makeCommandVoid1
            (*this, &Event::addSignal, docstring));

        docstring =
          "\n"
          "    Get list of signals\n";
        addCommand ("list", new command::Getter<Event, std::string>
            (*this, &Event::getSignalsByName, docstring));

        docstring =
          "\n"
          "    Triggers an event only when condition goes from False to True\n";
        addCommand ("setOnlyUp", new command::Setter<Event, bool>
            (*this, &Event::setOnlyUp, docstring));
      }

      ~Event () {}

      /// Header documentation of the python class
      virtual std::string getDocString () const
      {
        return
          "Send an event when the input changes\n\n"
          "  The signal triggered is called whenever the condition is satisfied.\n";
      }

      void addSignal (const std::string& signal)
      {
        std::istringstream iss (signal);
        triggers.push_back(&PoolStorage::getInstance()->getSignal (iss));
      }

      // Returns the Python string representation of the list of signal names.
      std::string getSignalsByName () const
      {
        std::ostringstream oss;
        oss << "(";
        for (Triggers_t::const_iterator _sig = triggers.begin();
            _sig != triggers.end(); ++_sig)
          oss << '\'' << (*_sig)->getName() << "\', ";
        oss << ")";
        return oss.str();
      }

      void setOnlyUp (const bool& up)
      {
        onlyUp_ = up;
      }

      private:
      typedef SignalBase<int>* Trigger_t;
      typedef std::vector<Trigger_t> Triggers_t;

      bool& check (bool& ret, const int& time);

      Signal <bool, int> checkSOUT;

      Triggers_t triggers;
      SignalPtr <bool, int> conditionSIN;

      bool lastVal_, onlyUp_;
    };
  } // namespace sot
} // namespace dynamicgraph
#endif // __SOT_EVENT_H__
