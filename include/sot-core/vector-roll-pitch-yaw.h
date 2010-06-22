/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vector-roll-pitch-yaw.h
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

#ifndef __SOT_VECTOR_ROLLPITCHYAW_H__
#define __SOT_VECTOR_ROLLPITCHYAW_H__

/* --- SOT --- */
#include <sot-core/vector-rotation.h>
#include <sot-core/sot-core-api.h>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
namespace sot {

class SOT_CORE_EXPORT VectorRollPitchYaw
: public sotVectorRotation
{
 public: 

  VectorRollPitchYaw( void ) : sotVectorRotation() { }
  virtual ~VectorRollPitchYaw( void ) { }

  virtual sotVectorRotation& fromMatrix( const MatrixRotation& rot );
  virtual MatrixRotation& toMatrix( MatrixRotation& rot ) const;

};
    
} // namespace sot

#endif /* #ifndef __SOT_VECTOR_ROLLPITCHYAW_H__ */
