/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Gepetto, LAAS-CNRS, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotTaskUnilateral.h
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
#include <sot-core/sotFeatureAbstract.h>
#include <sot-core/sotFlags.h>
#include <sot-core/sotTask.h>

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


class SOTTASKUNILATERAL_EXPORT sotTaskUnilateral
: public sotTask
{
 protected:
  std::list< sotFeatureAbstract* > featureList;

 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  sotTaskUnilateral( const std::string& n );

  /* --- COMPUTATION --- */
  sotVectorMultiBound& computeTaskUnilateral( sotVectorMultiBound& res,int time );


  /* --- SIGNALS ------------------------------------------------------------ */
 public:

  SignalPtr< ml::Vector,int > positionSIN;
  SignalPtr< ml::Vector,int > referenceInfSIN;
  SignalPtr< ml::Vector,int > referenceSupSIN;
  SignalPtr< double,int > dtSIN;

  /* --- DISPLAY ------------------------------------------------------------ */
  void display( std::ostream& os ) const;
};





#endif /* #ifndef __SOT_TASKUNILATERAL_H__ */


