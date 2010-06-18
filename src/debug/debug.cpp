/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic, 2005
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      debug.h
 * Project:   STack of Tasks
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
 * Macro de trace et de debugage
 *
 *   - TRACAGE:    TRACE et ERROR_TRACE fonctionnent comme des printf
 * avec retour chariot en fin de fonction.
 *                 CERROR et CTRACE fonctionnent comme les flux de sortie
 * C++ cout et cerr.
 *   - DEBUGAGE:   DEBUG_TRACE(niv,  et DERROR_TRACE(niv, fonctionnent
 * comme des printf, n'imprimant que si le niveau de trace 'niv' est
 * superieur au mode de debugage VP_DEBUG_MODE.
 *                 CDEBUG(niv) fonctionne comme le flux de sortie C++ cout.
 *                 DEBUG_ENABLE(niv) vaut 1 ssi le niveau de tracage 'niv'
 * est superieur au  mode de debugage DEBUG_MODE. Il vaut 0 sinon.
 *   - PROG DEFENSIVE: DEFENSIF(a) vaut a ssi le mode defensif est active,
 * et vaut 0 sinon.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <sot-core/debug.h>
#include <fstream>
#include <ios>

using namespace sot;

#ifdef WIN32
const char * DebugTrace::DEBUG_FILENAME_DEFAULT = "c:/tmp/traces.txt";
#else	/*WIN32*/
const char * DebugTrace::DEBUG_FILENAME_DEFAULT = "/tmp/traces.txt";
#endif	/*WIN32*/



#ifdef VP_DEBUG
 #ifdef WIN32
  std::ofstream debugfile( "C:/tmp/traces.txt", std::ios::trunc&std::ios::out );
 #else	/*WIN32*/
  std::ofstream debugfile( "/tmp/traces.txt", std::ios::trunc&std::ios::out );
#endif	/*WIN32*/
#else
std::ofstream debugfile; //( "/dev/null", std::ios::trunc&std::ios::out );
 class __sotDebug_init
 {
 public:
   __sotDebug_init( void ) 
     { debugfile.setstate( std::ios::failbit ) ; /* debugfile.close(); */ }
 };
 __sotDebug_init __sotDebug_initialisator;

#endif


void DebugTrace::openFile( const char * filename )
{
  if( debugfile.good()&&debugfile.is_open() ) debugfile.close();
  debugfile.clear();
  debugfile.open( filename, std::ios::trunc&std::ios::out );
  //std::cout << filename << debugfile.good() << debugfile.is_open() << std::endl;
}

void DebugTrace::closeFile( const char * filename )
{
  if( debugfile.good()&&debugfile.is_open() ) { debugfile.close(); }
  debugfile.setstate( std::ios::failbit ) ;
}


//DebugTrace sotDEBUGFLOW(std::cout);
//DebugTrace sotERRORFLOW(std::cerr);

/// Debug objects instances
namespace sot {
DebugTrace sotDEBUGFLOW(debugfile);
DebugTrace sotERRORFLOW(debugfile);
}
