/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotReader.h
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



#ifndef __SOT_TRACER_H__
#define __SOT_TRACER_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* STD */
#include <string>
#include <vector>
#include <list>
#include <boost/function.hpp>
#include <fstream>

/* SOT & DG*/
#include <dynamic-graph/signal-base.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/exception-traces.h>
#include <sot-core/flags.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (reader_EXPORTS)
#    define SOTREADER_EXPORT __declspec(dllexport)
#  else  
#    define SOTREADER_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTREADER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- TRACER ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dg = dynamicgraph;

class SOTREADER_EXPORT sotReader
: public dg::Entity
{
 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) { return CLASS_NAME; }

 public: 

  dg::SignalPtr< sot::Flags,int > selectionSIN;
  dg::SignalTimeDependent<ml::Vector,int> vectorSOUT;
  dg::SignalTimeDependent<ml::Matrix,int> matrixSOUT;

 public:
  sotReader( const std::string n );
  virtual ~sotReader( void ){}

  void load( const std::string& filename );
  void clear( void );
  void rewind( void );

 protected:

  typedef std::list< std::vector<double> > DataType;
  DataType dataSet;
  DataType::const_iterator currentData;
  bool iteratorSet;

  unsigned int nbRows,nbCols;

  ml::Vector& getNextData( ml::Vector& res, const unsigned int time );
  ml::Matrix& getNextMatrix( ml::Matrix& res, const unsigned int time );

 public:
  /* --- PARAMS --- */
  void display( std::ostream& os ) const;
  virtual void commandLine( const std::string& cmdLine
			    ,std::istringstream& cmdArgs
			    ,std::ostream& os );
};





#endif /* #ifndef __SOT_TRACER_H__ */


