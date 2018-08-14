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
namespace sot {

/*!
  \class PeriodicCall
*/
class SOT_CORE_EXPORT PeriodicCall
{
 protected:
  struct SignalToCall
  {
    dynamicgraph::SignalBase<int>* signal;
    unsigned int downsamplingFactor;

    SignalToCall()
    {
      signal = NULL;
      downsamplingFactor = 1;
    }

    SignalToCall(dynamicgraph::SignalBase<int>* s, unsigned int df=1)
    {
      signal = s;
      downsamplingFactor = df;
    }
  };

  typedef std::map< std::string,SignalToCall > SignalMapType;
  SignalMapType signalMap;

  int innerTime;

  /* --- FUNCTIONS ------------------------------------------------------------ */
 public:
  PeriodicCall( void );
  virtual ~PeriodicCall( void ) {}

  void addDownsampledSignal( const std::string &name, dynamicgraph::SignalBase<int>& sig, const unsigned int& downsamplingFactor );
  void addDownsampledSignal( const std::string& sigpath, const unsigned int& downsamplingFactor );

  void addSignal( const std::string &name, dynamicgraph::SignalBase<int>& sig );
  void addSignal( const std::string& args );
  void rmSignal( const std::string &name );

  void runSignals( const int& t );
  void run( const int& t );

  void clear( void ) { signalMap.clear(); }

  void display( std::ostream& os ) const;
  void addSpecificCommands( dynamicgraph::Entity& ent,
			    dynamicgraph::Entity::CommandMap_t& commap,
			    const std::string & prefix = "" );
};



} // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __SOT_PERIODICCALL_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
