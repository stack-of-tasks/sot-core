/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      SeqPlay.h
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef __SOT_SEQPLAY_HH
#define __SOT_SEQPLAY_HH

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* -- MaaL --- */
#include <MatrixAbstractLayer/boost.h>
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




