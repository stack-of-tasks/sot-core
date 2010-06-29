/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Gepetto, LAAS-CNRS, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      task-unilateral.h
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
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef __SOT_TASKUNILATERAL_H__
#define __SOT_TASKUNILATERAL_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* STD */
#include <string>

/* SOT */
#include <sot-core/feature-abstract.h>
#include <sot-core/flags.h>
#include <sot-core/task.h>

#include <sot-core/exception-task.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (sotTaskUnilateral_EXPORTS)
#    define SOTTASKUNILATERAL_EXPORT __declspec(dllexport)
#  else
#    define SOTTASKUNILATERAL_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTTASKUNILATERAL_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

class SOTTASKUNILATERAL_EXPORT TaskUnilateral
: public Task
{
 protected:
  std::list< FeatureAbstract* > featureList;

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  TaskUnilateral( const std::string& n );

  /* --- COMPUTATION --- */
  VectorMultiBound& computeTaskUnilateral( VectorMultiBound& res,int time );


  /* --- SIGNALS ------------------------------------------------------------ */
 public:

  dg::SignalPtr< ml::Vector,int > positionSIN;
  dg::SignalPtr< ml::Vector,int > referenceInfSIN;
  dg::SignalPtr< ml::Vector,int > referenceSupSIN;
  dg::SignalPtr< double,int > dtSIN;

  /* --- DISPLAY ------------------------------------------------------------ */
  void display( std::ostream& os ) const;
};

} // namespace sot



#endif /* #ifndef __SOT_TASKUNILATERAL_H__ */


