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
class VectorUTheta;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
class SOT_CORE_EXPORT MatrixRotation
: public ml::Matrix
{

 public: 

  MatrixRotation( void ) : ml::Matrix(3,3) { setIdentity(); }
  ~MatrixRotation( void ) { }

  void fromVector( VectorUTheta& );
  MatrixRotation& operator= ( VectorUTheta&th ) { fromVector(th); return *this; } 
};
    
} // namespace sot
#endif /* #ifndef __SOT_MATRIX_ROTATION_H__ */






