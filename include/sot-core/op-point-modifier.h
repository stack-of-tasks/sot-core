/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      op-point-modifier.h
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

#ifndef __SOT_OP_POINT_MODIFIOR_H__
#define __SOT_OP_POINT_MODIFIOR_H__

#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/debug.h>
#include <sot-core/matrix-homogeneous.h>

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (OpPointModifier_EXPORTS)
#    define SOTOPPOINTMODIFIER_EXPORT __declspec(dllexport)
#  else  
#    define SOTOPPOINTMODIFIER_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTOPPOINTMODIFIER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

class SOTOPPOINTMODIFIER_EXPORT OpPointModifier
: public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
  
  MatrixHomogeneous transformation;

 public:

  dg::SignalPtr<ml::Matrix,int> jacobianSIN;
  dg::SignalPtr<MatrixHomogeneous,int> positionSIN;
  
  dg::SignalTimeDependent<ml::Matrix,int> jacobianSOUT;
  dg::SignalTimeDependent<MatrixHomogeneous,int> positionSOUT;

public:
  OpPointModifier( const std::string& name );

  virtual ~OpPointModifier( void ){}

  ml::Matrix& computeJacobian( ml::Matrix& res,const int& time );
  MatrixHomogeneous& computePosition( MatrixHomogeneous& res,const int& time );
  void setTransformation( const MatrixHomogeneous& tr );

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs, 
			    std::ostream& os );

};

} // namespace sot

#endif //  __SOT_OP_POINT_MODIFIOR_H__ 
