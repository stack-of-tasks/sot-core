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
#  if defined (sotOpPointModifior_EXPORTS)
#    define SOTOPPOINTMODIFIOR_EXPORT __declspec(dllexport)
#  else  
#    define SOTOPPOINTMODIFIOR_EXPORT __declspec(dllimport)
#  endif 
#else
#  define SOTOPPOINTMODIFIOR_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- VECTOR ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

class SOTOPPOINTMODIFIOR_EXPORT OpPointModifior
: public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
  
  MatrixHomogeneous transformation;

 public:

  dg::SignalPtr<ml::Matrix,int> jacobianSIN;
  dg::SignalPtr<MatrixHomogeneous,int> positionSIN;
  
  dg::SignalTimeDependant<ml::Matrix,int> jacobianSOUT;
  dg::SignalTimeDependant<MatrixHomogeneous,int> positionSOUT;

public:
  OpPointModifior( const std::string& name );

  virtual ~OpPointModifior( void ){}

  ml::Matrix& computeJacobian( ml::Matrix& res,const int& time );
  MatrixHomogeneous& computePosition( MatrixHomogeneous& res,const int& time );
  void setTransformation( const MatrixHomogeneous& tr );

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs, 
			    std::ostream& os );

};

} // namespace sot

#endif //  __SOT_OP_POINT_MODIFIOR_H__ 
