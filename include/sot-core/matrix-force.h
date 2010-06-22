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


class MatrixHomogeneous;
class MatrixTwist;


class SOT_CORE_EXPORT MatrixForce
: public ml::Matrix
{

 public: 

  MatrixForce( void ) : ml::Matrix(6,6) { setIdentity(); }
  ~MatrixForce( void ) { }
  explicit MatrixForce( const MatrixHomogeneous& M ) 
    : ml::Matrix(6,6) 
    { buildFrom(M); }

  MatrixForce& buildFrom( const MatrixHomogeneous& trans );

  MatrixForce& operator=( const ml::Matrix& );
  MatrixForce&
    inverse( MatrixForce& invMatrix ) const ;
  inline MatrixForce inverse( void )  const 
    { MatrixForce Ainv; return inverse(Ainv); }

  MatrixTwist& transpose( MatrixTwist& Vt ) const;
  MatrixTwist transpose( void ) const;

 };

} // namespace sot
    

#endif /* #ifndef __SOT_MATRIX_FORCE_H__ */






