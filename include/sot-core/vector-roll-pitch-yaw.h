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
class SOT_CORE_EXPORT sotVectorRollPitchYaw
: public sotVectorRotation
{
 public: 

  sotVectorRollPitchYaw( void ) : sotVectorRotation() { }
  virtual ~sotVectorRollPitchYaw( void ) { }

  virtual sotVectorRotation& fromMatrix( const sotMatrixRotation& rot );
  virtual sotMatrixRotation& toMatrix( sotMatrixRotation& rot ) const;

};
    


#endif /* #ifndef __SOT_VECTOR_ROLLPITCHYAW_H__ */
