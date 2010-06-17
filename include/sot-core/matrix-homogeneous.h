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
namespace ml = maal::boost;

class sotMatrixRotation;

#include <sot-core/sot-core-api.h>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
class SOT_CORE_EXPORT sotMatrixHomogeneous
: public ml::Matrix
{

 public: 

  sotMatrixHomogeneous( void ) : ml::Matrix(4,4) { setIdentity(); }
  ~sotMatrixHomogeneous( void ) { }

  sotMatrixHomogeneous& buildFrom( const sotMatrixRotation& rot, const ml::Vector& trans );
  sotMatrixRotation& extract( sotMatrixRotation& rot ) const;
  ml::Vector& extract( ml::Vector& trans ) const;

  sotMatrixHomogeneous& operator=( const ml::Matrix& );

  sotMatrixHomogeneous&
    inverse( sotMatrixHomogeneous& invMatrix ) const ;
  inline sotMatrixHomogeneous inverse( void )  const 
    { sotMatrixHomogeneous Ainv; return inverse(Ainv); }

  ml::Vector& multiply( const ml::Vector& v1,ml::Vector& res ) const;
  inline ml::Vector multiply( const ml::Vector& v1 )  const
    { ml::Vector res; return multiply(v1,res); }
  
  using  ml::Matrix::multiply;
   

 


 };
    



#endif /* #ifndef __SOT_MATRIX_HOMOGENEOUS_H__ */






