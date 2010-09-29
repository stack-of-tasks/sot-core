/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_NeckLimitation_H__
#define __SOT_NeckLimitation_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;

/* SOT */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/task-abstract.h>

/* STD */
#include <string>
#include <map>
#include <list>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (neck_limitation_EXPORTS)
#    define NeckLimitation_EXPORT __declspec(dllexport)
#  else  
#    define NeckLimitation_EXPORT __declspec(dllimport)
#  endif 
#else
#  define NeckLimitation_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

namespace dg = dynamicgraph;

class NeckLimitation_EXPORT NeckLimitation
:public dg::Entity
{
 public:
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 protected:

  unsigned int panRank,tiltRank;
  static const unsigned int PAN_RANK_DEFAULT;
  static const unsigned int TILT_RANK_DEFAULT;


  /* The limitation is: sgn.Tilt >= Pan.alpha + beta, with alpha the linear
  * coefficient and beta the affine one, and sgn is +1 or -1. */
  double coeffLinearPan,coeffAffinePan;
  double signTilt; 
  static const double COEFF_LINEAR_DEFAULT;
  static const double COEFF_AFFINE_DEFAULT;
  static const double SIGN_TILT_DEFAULT;

 public: /* --- CONSTRUCTION --- */

  NeckLimitation( const std::string& name );
  virtual ~NeckLimitation( void );

 public: /* --- SIGNAL --- */
  dg::SignalPtr<ml::Vector,int> jointSIN;
  dg::SignalTimeDependent<ml::Vector,int> jointSOUT;

 public: /* --- FUNCTIONS --- */
  ml::Vector& computeJointLimitation( ml::Vector& jointLimited,const int& timeSpec );
  
 public: /* --- PARAMS --- */
  virtual void display( std::ostream& os ) const; 
  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs,
			    std::ostream& os );
};


} // namespace sot



#endif // #ifndef __SOT_NeckLimitation_H__

