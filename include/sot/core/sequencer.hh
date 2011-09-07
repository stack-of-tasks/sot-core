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

#ifndef __SOT_SOTSEQUENCER_H__
#define __SOT_SOTSEQUENCER_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <sot/core/task-abstract.hh>

/* STD */
#include <string>
#include <map>
#include <list>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (sequencer_EXPORTS)
#    define SOTSEQUENCER_EXPORT __declspec(dllexport)
#  else
#    define SOTSEQUENCER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSEQUENCER_EXPORT
#endif

namespace dynamicgraph {
  namespace sot {

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */

    class Sot;

    class SOTSEQUENCER_EXPORT Sequencer
      :public dynamicgraph::Entity
      {
	DYNAMIC_GRAPH_ENTITY_DECL();
      public:
	class sotEventAbstract
	{
	public:
	  enum sotEventType
	  {
	    EVENT_ADD
	    ,EVENT_RM
	    ,EVENT_CMD
	  };
	protected:
	  std::string name;
	  void setName( const std::string& name_ ) { name = name_; }
	  int eventType;
	public:
	sotEventAbstract( const std::string & name ) : name(name) {};
	  virtual ~sotEventAbstract( void ) {}
	  virtual const std::string& getName() const { return name; }
	  int getEventType(  ) const { return eventType; }
	  virtual void operator() ( Sot* sotPtr ) = 0;
	  virtual void display( std::ostream& os ) const { os << name; }
	};

      protected:
	Sot* sotPtr;
	typedef std::list< sotEventAbstract* > TaskList;
	typedef std::map< unsigned int,TaskList > TaskMap;

	TaskMap taskMap;
	/* All the events are counting wrt to this t0. If t0 is -1, it
	 * is set to the first time of trig.    */
	int timeInit;
	bool playMode;
	std::ostream* outputStreamPtr;
	bool noOutput; /*! if true, display nothing standard output on except errors*/

      public: /* --- CONSTRUCTION --- */

	Sequencer( const std::string& name );
	virtual ~Sequencer( void );

      public: /* --- TASK MANIP --- */

	void setSotRef( Sot* sot ) { sotPtr = sot; }
	void addTask( sotEventAbstract* task,const unsigned int time );
	void rmTask( int eventType, const std::string& name,const unsigned int time );
	void clearAll( );

      public: /* --- SIGNAL --- */
	dynamicgraph::SignalTimeDependent<int,int> triggerSOUT;

      public: /* --- FUNCTIONS --- */
	int& trigger( int& dummy,const int& time );

      public: /* --- PARAMS --- */
	virtual void display( std::ostream& os ) const;
	virtual void commandLine( const std::string& cmdLine,
				  std::istringstream& cmdArgs,
				  std::ostream& os );

      };
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __SOT_SOTSEQUENCER_H__

