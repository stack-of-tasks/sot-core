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

#ifndef __SOT_SWITCH_H__
# define __SOT_SWITCH_H__

#include <iostream>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-getter.h>

#include <sot/core/config.hh>

namespace dynamicgraph {
  namespace sot {
    /// Switch
    template <typename Value, typename Time = int>
    class SOT_CORE_DLLAPI Switch : public dynamicgraph::Entity
    {
      DYNAMIC_GRAPH_ENTITY_DECL();

      Switch (const std::string& name) :
        Entity (name),
        selectionSIN(NULL,"Switch("+name+")::input(int)::selection"),
        boolSelectionSIN(NULL,"Switch("+name+")::input(bool)::boolSelection"),
        signalSOUT  ("Switch("+name+")::output(" + typeName() + ")::sout")
      {
        signalSOUT.setFunction (boost::bind (&Switch::signal, this, _1, _2));
        signalRegistration (selectionSIN << boolSelectionSIN << signalSOUT);

        using command::makeCommandVoid1;
        std::string docstring =
          "\n"
          "    Set number of input signals\n";
        addCommand ("setSignalNumber", makeCommandVoid1
            (*this, &Switch::setSignalNumber, docstring));
      }

      ~Switch () {}

      /// Header documentation of the python class
      virtual std::string getDocString () const
      {
        return
          "Dynamically select a given signal based on a input information.\n";
      }

      void setSignalNumber (const int& n)
      {
        assert (n>=0);
        const std::size_t oldSize = signals.size();
        for (std::size_t i = n; i < oldSize; ++i)
        {
          std::ostringstream oss; oss << "sin" << i;
          signalDeregistration(oss.str());
          delete signals[i];
        }
        signals.resize(n,NULL);
        
        for (std::size_t i = oldSize; i < (std::size_t)n; ++i)
        {
          assert (signals[i]==NULL);
          std::ostringstream oss;
          oss << "Switch("<< getName()<< ")::input(" << typeName() << ")::sin" << i;
          signals[i] = new Signal_t (NULL,oss.str());
          signalRegistration(*signals[i]);
        }
      }

      private:
      typedef SignalPtr<Value, Time> Signal_t;
      typedef std::vector<Signal_t*> Signals_t;

      static const std::string& typeName ();

      Value& signal (Value& ret, const Time& time)
      {
        int sel;
        if (selectionSIN.isPlugged()) {
          sel = selectionSIN (time);
        } else {
          const bool& b = boolSelectionSIN(time);
          sel = b ? 1 : 0;
        }
        if (sel < 0 || sel >= int(signals.size()))
          throw std::runtime_error ("Signal selection is out of range.");

        ret = (*signals[sel]) (time);
        return ret;
      }

      Signals_t signals;
      SignalPtr <int, Time> selectionSIN;
      SignalPtr <bool, Time> boolSelectionSIN;

      Signal <Value, Time> signalSOUT;
    };
  } // namespace sot
} // namespace dynamicgraph
#endif // __SOT_SWITCH_H__
