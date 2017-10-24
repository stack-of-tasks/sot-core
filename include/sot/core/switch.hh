/*
 * Copyright 2017-, Rohan Budhiraja, CNRS
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_SWITCH_H__
#define __SOT_SWITCH_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <dynamic-graph/command-bind.h>
#include <sot/core/pool.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>

/* STD */
#include <string>

namespace dynamicgraph { 
  namespace sot {
    
    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */
    using dynamicgraph::Entity;
    using dynamicgraph::command::makeCommandVoid0;
    using dynamicgraph::command::docCommandVoid0;

    class Switch : public Entity
    {
     
    public: /* --- SIGNAL --- */
      DYNAMIC_GRAPH_ENTITY_DECL();
      dynamicgraph::SignalTimeDependent<bool,int> outSOUT;
      
    protected:
      bool signalOutput;
      void turnOn(){ signalOutput = true; }

      void turnOff(){ signalOutput = false; }

      bool& switchOutput(bool& res, int){ res = signalOutput; return res; }

    public:
      Switch( const std::string& name )
	: Entity(name)
	,outSOUT( boost::bind(&Switch::switchOutput,this,_1,_2),
                  sotNOSIGNAL,"Switch("+name+")::output(bool)::out")
      {
        signalOutput = false;
        signalRegistration (outSOUT );
        addCommand ("turnOn",
                    makeCommandVoid0 (*this, &Switch::turnOn,
                                      docCommandVoid0 ("Turn on the switch")));
        addCommand ("turnOff",
                    makeCommandVoid0 (*this, &Switch::turnOff,
                                      docCommandVoid0 ("Turn off the switch")));
      }
      
        virtual ~Switch( void ) {};

};
} /* namespace sot */
} /* namespace dynamicgraph */



#endif // #ifndef __SOT_SWITCH_H__
