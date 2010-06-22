/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vector-quaternion.h
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

#ifndef __SOT_VECTOR_QUATERNION_H__
#define __SOT_VECTOR_QUATERNION_H__

/* --- SOT --- */
#include <sot-core/vector-rotation.h>
#include <sot-core/sot-core-api.h>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace sot {

class SOT_CORE_EXPORT VectorQuaternion
: public sotVectorRotation
{
 public: 

  VectorQuaternion( void ) : sotVectorRotation() { ml::Vector::resize(4); }
  virtual ~VectorQuaternion( void ) { }

  virtual sotVectorRotation& fromMatrix( const MatrixRotation& rot );
  virtual MatrixRotation& toMatrix( MatrixRotation& rot ) const;

  sotVectorRotation& fromVector( const VectorUTheta& ut );

  VectorQuaternion& conjugate(VectorQuaternion& res) const;
  VectorQuaternion& multiply(const VectorQuaternion& q2, VectorQuaternion& res) const;

};
    
}


#endif /* #ifndef __SOT_VECTOR_QUATERNION_H__ */
