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

class SOTOPPOINTMODIFIOR_EXPORT sotOpPointModifior
: public Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }
  
  sotMatrixHomogeneous transformation;

 public:

  SignalPtr<ml::Matrix,int> jacobianSIN;
  SignalPtr<sotMatrixHomogeneous,int> positionSIN;
  
  SignalTimeDependant<ml::Matrix,int> jacobianSOUT;
  SignalTimeDependant<sotMatrixHomogeneous,int> positionSOUT;

public:
  sotOpPointModifior( const std::string& name );

  virtual ~sotOpPointModifior( void ){}

  ml::Matrix& computeJacobian( ml::Matrix& res,const int& time );
  sotMatrixHomogeneous& computePosition( sotMatrixHomogeneous& res,const int& time );
  void setTransformation( const sotMatrixHomogeneous& tr );

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs, 
			    std::ostream& os );

};

} // namespace sot

#endif //  __SOT_OP_POINT_MODIFIOR_H__ 
