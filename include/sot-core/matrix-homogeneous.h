/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      matrix-homogeneous.h
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

#ifndef __SOT_MATRIX_HOMOGENEOUS_H__
#define __SOT_MATRIX_HOMOGENEOUS_H__


/* --- Matrix --- */
#include <MatrixAbstractLayer/boost.h>
#include <sot-core/sot-core-api.h>
namespace ml = maal::boost;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace sot {

class MatrixRotation;

class SOT_CORE_EXPORT MatrixHomogeneous
: public ml::Matrix
{

 public: 

  MatrixHomogeneous( void ) : ml::Matrix(4,4) { setIdentity(); }
  ~MatrixHomogeneous( void ) { }

  MatrixHomogeneous& buildFrom( const MatrixRotation& rot, const ml::Vector& trans );
  MatrixRotation& extract( MatrixRotation& rot ) const;
  ml::Vector& extract( ml::Vector& trans ) const;

  MatrixHomogeneous& operator=( const ml::Matrix& );

  MatrixHomogeneous&
    inverse( MatrixHomogeneous& invMatrix ) const ;
  inline MatrixHomogeneous inverse( void )  const 
    { MatrixHomogeneous Ainv; return inverse(Ainv); }

  ml::Vector& multiply( const ml::Vector& v1,ml::Vector& res ) const;
  inline ml::Vector multiply( const ml::Vector& v1 )  const
    { ml::Vector res; return multiply(v1,res); }
  
  using  ml::Matrix::multiply;
   

 


 };
    
} // namespace sot


#endif /* #ifndef __SOT_MATRIX_HOMOGENEOUS_H__ */






