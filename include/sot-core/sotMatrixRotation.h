/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotMatrixRotation.h
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

#ifndef __SOT_MATRIX_ROTATION_H__
#define __SOT_MATRIX_ROTATION_H__


/* --- Matrix --- */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

class sotVectorUTheta;

#include <sot-core/sot-core-api.h>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
class SOT_CORE_EXPORT sotMatrixRotation
: public ml::Matrix
{

 public: 

  sotMatrixRotation( void ) : ml::Matrix(3,3) { setIdentity(); }
  ~sotMatrixRotation( void ) { }

  void fromVector( sotVectorUTheta& );
  sotMatrixRotation& operator= ( sotVectorUTheta&th ) { fromVector(th); return *this; } 
};
    

#endif /* #ifndef __SOT_MATRIX_ROTATION_H__ */






