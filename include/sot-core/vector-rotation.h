/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vector-rotation.h
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

#ifndef __SOT_VECTOR_ROTATION_H__
#define __SOT_VECTOR_ROTATION_H__

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* --- SOT --- */
#include <sot-core/matrix-rotation.h>
#include <sot-core/sot-core-api.h>

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

class SOT_CORE_EXPORT sotVectorRotation
: public ml::Vector
{
 public: 

  sotVectorRotation( void ) : ml::Vector(3) { fill(0.); } 
  virtual ~sotVectorRotation( void ) { }

  virtual sotVectorRotation& fromMatrix( const MatrixRotation& rot ) = 0;
  virtual MatrixRotation& toMatrix( MatrixRotation& rot ) const = 0;
};

} // namespace sot

#endif /* #ifndef __SOT_VECTOR_ROTATION_H__ */




