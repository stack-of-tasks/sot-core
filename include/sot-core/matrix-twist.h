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


class MatrixHomogeneous;
class MatrixForce;

class SOT_CORE_EXPORT MatrixTwist
: public ml::Matrix
{

 public: 

  MatrixTwist( void ) : ml::Matrix(6,6) { setIdentity(); }
  ~MatrixTwist( void ) { }
  explicit MatrixTwist( const MatrixHomogeneous& M ) 
    : ml::Matrix(6,6) 
    { buildFrom(M); }

  MatrixTwist& buildFrom( const MatrixHomogeneous& trans );

  MatrixTwist& operator=( const ml::Matrix& );
  MatrixTwist&
    inverse( MatrixTwist& invMatrix ) const ;
  inline MatrixTwist inverse( void )  const 
    { MatrixTwist Ainv; return inverse(Ainv); }

  MatrixForce& transpose( MatrixForce& Vt ) const;
  MatrixForce transpose( void ) const;
 };

} // namespace sot

#endif /* #ifndef __SOT_MATRIX_TWIST_H__ */






