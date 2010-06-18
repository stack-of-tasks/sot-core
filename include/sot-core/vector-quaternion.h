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

class SOT_CORE_EXPORT sotVectorQuaternion
: public sotVectorRotation
{
 public: 

  sotVectorQuaternion( void ) : sotVectorRotation() { ml::Vector::resize(4); }
  virtual ~sotVectorQuaternion( void ) { }

  virtual sotVectorRotation& fromMatrix( const sotMatrixRotation& rot );
  virtual sotMatrixRotation& toMatrix( sotMatrixRotation& rot ) const;

  sotVectorRotation& fromVector( const sotVectorUTheta& ut );

  sotVectorQuaternion& conjugate(sotVectorQuaternion& res) const;
  sotVectorQuaternion& multiply(const sotVectorQuaternion& q2, sotVectorQuaternion& res) const;

};
    
}


#endif /* #ifndef __SOT_VECTOR_QUATERNION_H__ */
