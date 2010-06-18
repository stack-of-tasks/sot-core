/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      matrix-force.h
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

#ifndef __SOT_MATRIX_FORCE_H__
#define __SOT_MATRIX_FORCE_H__


/* --- Matrix --- */
#include <MatrixAbstractLayer/boost.h>
#include <sot-core/sot-core-api.h>
namespace ml = maal::boost;


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {


class sotMatrixHomogeneous;
class sotMatrixTwist;


class SOT_CORE_EXPORT sotMatrixForce
: public ml::Matrix
{

 public: 

  sotMatrixForce( void ) : ml::Matrix(6,6) { setIdentity(); }
  ~sotMatrixForce( void ) { }
  explicit sotMatrixForce( const sotMatrixHomogeneous& M ) 
    : ml::Matrix(6,6) 
    { buildFrom(M); }

  sotMatrixForce& buildFrom( const sotMatrixHomogeneous& trans );

  sotMatrixForce& operator=( const ml::Matrix& );
  sotMatrixForce&
    inverse( sotMatrixForce& invMatrix ) const ;
  inline sotMatrixForce inverse( void )  const 
    { sotMatrixForce Ainv; return inverse(Ainv); }

  sotMatrixTwist& transpose( sotMatrixTwist& Vt ) const;
  sotMatrixTwist transpose( void ) const;

 };

} // namespace sot
    

#endif /* #ifndef __SOT_MATRIX_FORCE_H__ */






