/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      PeriodicCall.h
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


#ifndef __SOT_PERIODICCALL_HH__
#define __SOT_PERIODICCALL_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <dynamic-graph/signal-base.h>
#include <sot-core/sot-core-api.h>
/* STD */
#include <list>
#include <map>
#include <string>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

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

  /* --- FUNCTIONS ------------------------------------------------------------ */
 public:
  PeriodicCall( void );
  virtual ~PeriodicCall( void ) {}

  void addSignal( const std::string &name, dynamicgraph::SignalBase<int>& sig );
  void addSignal( std::istringstream& args );
  void rmSignal( const std::string &name );
  void rmSignal( std::istringstream& args );

  void addCmd( std::istringstream& args );
  void rmCmd( std::istringstream& args );

  void runSignals( const int& t );
  void runCmds( void );
  void run( const int& t );
  
  void clear( void ) { signalMap.clear(); cmdList.clear(); }

  void display( std::ostream& os ) const;
  bool commandLine( const std::string& cmdLine,
		    std::istringstream& cmdArgs,
		    std::ostream& os );
};



} // namespace sot



#endif // #ifndef __SOT_PERIODICCALL_HH__

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
