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

#ifndef __SOT_PERIODICCALL_ENTITY_HH__
#define __SOT_PERIODICCALL_ENTITY_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <dynamic-graph/entity.h>
#include <sot/core/periodic-call.hh>
#include <sot/core/periodic-call-entity.hh>
#include <dynamic-graph/all-signals.h>
/* STD */
#include <list>
#include <map>
#include <string>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (periodic_call_entity_EXPORTS)
#    define PeriodicCallEntity_EXPORT __declspec(dllexport)
#  else
#    define PeriodicCallEntity_EXPORT __declspec(dllimport)
#  endif
#else
#  define PeriodicCallEntity_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

namespace dg = dynamicgraph;

/*!
  \class PeriodicCallEntity

  The entity remembers a stack of signal and command to be executed or
  refreshed at each iteration. The update is trigered by the triger signal.
  If the trigerOnce is called, the stacks are flushed after the execution.
*/
class PeriodicCallEntity_EXPORT PeriodicCallEntity
: public dg::Entity, protected sot::PeriodicCall
{

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  dg::Signal<int,int> triger;
  dg::Signal<int,int> trigerOnce;

  int& trigerCall( int& dummy,const int & time );
  int& trigerOnceCall( int& dummy,const int & time );

  /* --- FUNCTIONS ------------------------------------------------------------ */
 public:
  PeriodicCallEntity( const std::string& name );
  virtual ~PeriodicCallEntity( void ) {}

  virtual void display( std::ostream& os ) const;
  void commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );
} ;


} // ns dynamicgraph


#endif // #ifndef __SOT_PERIODICCALL_ENTITY_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
