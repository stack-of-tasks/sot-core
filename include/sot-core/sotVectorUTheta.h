/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotVectorUTheta.h
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

#ifndef __SOT_VECTOR_UTHETA_H__
#define __SOT_VECTOR_UTHETA_H__

/* --- SOT --- */
#include <sot-core/sotVectorRotation.h>
#include <sot-core/sot-core-api.h>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
class SOT_CORE_EXPORT sotVectorUTheta
: public sotVectorRotation
{
 public: 

  sotVectorUTheta( void ) : sotVectorRotation() { }
  virtual ~sotVectorUTheta( void ) { }

  virtual sotVectorRotation& fromMatrix( const sotMatrixRotation& rot );
  virtual sotMatrixRotation& toMatrix( sotMatrixRotation& rot ) const;

};
    


#endif /* #ifndef __SOT_VECTOR_UTHETA_H__ */
