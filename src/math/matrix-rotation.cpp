/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      MatrixRotation.cpp
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

#include <sot-core/matrix-rotation.h>
#include <sot-core/vector-utheta.h>

using namespace std;
using namespace sot;


void MatrixRotation::fromVector( VectorUTheta& vec )
{
  vec.toMatrix( *this );
}
  

