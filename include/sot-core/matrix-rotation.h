/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      matrix-rotation.h
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

#include <sot-core/sot-core-api.h>
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

namespace sot {
class sotVectorUTheta;

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
    
} // namespace sot
#endif /* #ifndef __SOT_MATRIX_ROTATION_H__ */






