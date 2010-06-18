/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      matrix-twist.h
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

#ifndef __SOT_MATRIX_TWIST_H__
#define __SOT_MATRIX_TWIST_H__


/* --- Matrix --- */
#include <MatrixAbstractLayer/boost.h>
#include <sot-core/sot-core-api.h>

namespace ml = maal::boost;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace sot {


class sotMatrixHomogeneous;
class sotMatrixForce;

class SOT_CORE_EXPORT sotMatrixTwist
: public ml::Matrix
{

 public: 

  sotMatrixTwist( void ) : ml::Matrix(6,6) { setIdentity(); }
  ~sotMatrixTwist( void ) { }
  explicit sotMatrixTwist( const sotMatrixHomogeneous& M ) 
    : ml::Matrix(6,6) 
    { buildFrom(M); }

  sotMatrixTwist& buildFrom( const sotMatrixHomogeneous& trans );

  sotMatrixTwist& operator=( const ml::Matrix& );
  sotMatrixTwist&
    inverse( sotMatrixTwist& invMatrix ) const ;
  inline sotMatrixTwist inverse( void )  const 
    { sotMatrixTwist Ainv; return inverse(Ainv); }

  sotMatrixForce& transpose( sotMatrixForce& Vt ) const;
  sotMatrixForce transpose( void ) const;
 };

} // namespace sot

#endif /* #ifndef __SOT_MATRIX_TWIST_H__ */






