/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotPeriodicCall.h
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


#ifndef __SOT_PERIODICCALL_ENTITY_HH__
#define __SOT_PERIODICCALL_ENTITY_HH__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


/* SOT */
#include <dynamic-graph/entity.h>
#include <sot-core/periodic-call.h>
#include <sot-core/periodic-call-entity.h>
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
