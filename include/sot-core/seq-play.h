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

#ifndef __SOT_SEQPLAY_HH
#define __SOT_SEQPLAY_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* -- MaaL --- */
#include <jrl/mal/boost.hh>
namespace ml= maal::boost;
/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>

#include <list>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (seq_play_EXPORTS)
#    define SOTSEQPLAY_EXPORT __declspec(dllexport)
#  else
#    define SOTSEQPLAY_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTSEQPLAY_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

class SOTSEQPLAY_EXPORT SeqPlay
:public dynamicgraph::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:

  typedef  std::list<ml::Vector> StateList;
  StateList stateList;
  StateList::iterator currPos; unsigned int currRank;
  bool init;
  int time;

 public:

  /* --- CONSTRUCTION --- */
  SeqPlay( const std::string& name );
  virtual ~SeqPlay( void ) { }

  void loadFile( const std::string& name );

  ml::Vector& getNextPosition( ml::Vector& pos, const int& time );

 public: /* --- DISPLAY --- */
  virtual void display( std::ostream& os ) const;
  SOTSEQPLAY_EXPORT friend std::ostream& operator<< ( std::ostream& os,const SeqPlay& r )
    { r.display(os); return os;}

 public: /* --- SIGNALS --- */

  //dynamicgraph::SignalPtr<ml::Vector,int> positionSIN;
  //dynamicgraph::SignalTimeDependant<ml::Vector,int> velocitySOUT;
  dynamicgraph::SignalTimeDependent<int,int> refresherSINTERN;
  dynamicgraph::SignalTimeDependent<ml::Vector,int> positionSOUT;

 public: /* --- COMMANDS --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
			    std::ostream& os );

};


} // namespace sot

#endif /* #ifndef __SOT_SEQPLAY_HH */




