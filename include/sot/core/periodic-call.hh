/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
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

#ifndef __SOT_PERIODICCALL_HH__
#define __SOT_PERIODICCALL_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <dynamic-graph/signal-base.h>
#include <dynamic-graph/entity.h>
#include <sot/core/api.hh>
/* STD */
#include <list>
#include <map>
#include <string>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
  namespace python {
    class Interpreter;
  }
}

namespace dynamicgraph {
namespace sot {

/*!
  \class PeriodicCall
*/
class SOT_CORE_EXPORT PeriodicCall
{
 protected:
  typedef std::map< std::string,dynamicgraph::SignalBase<int>* > SignalMapType;
  SignalMapType signalMap;

  typedef std::list< std::string > CmdListType;
  CmdListType cmdList;

  int innerTime;
  dynamicgraph::python::Interpreter * py_sh;

  /* --- FUNCTIONS ------------------------------------------------------------ */
 public:
  PeriodicCall( void );
  virtual ~PeriodicCall( void ) {}

  void addSignal( const std::string &name, dynamicgraph::SignalBase<int>& sig );
  void addSignal( const std::string& args );
  void rmSignal( const std::string &name );

  void addCmd( const std::string& args );
  void rmCmd( const std::string& args );

  void runSignals( const int& t );
  void runCmds( void );
  void run( const int& t );

  void clear( void ) { signalMap.clear(); cmdList.clear(); }

  void display( std::ostream& os ) const;
  bool commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );
  void addSpecificCommands( dynamicgraph::Entity& ent,
			    dynamicgraph::Entity::CommandMap_t& commap,
			    const std::string & prefix = "" );

  void setPyInterpreter( dynamicgraph::python::Interpreter * py_sh );
};



} // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __SOT_PERIODICCALL_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
