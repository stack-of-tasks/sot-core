/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
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

#ifndef __sot_core_VisualPointProjecter_H__
#define __sot_core_VisualPointProjecter_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (visual_point_projecter_EXPORTS)
#    define SOTVISUALPOINTPROJECTER_EXPORT __declspec(dllexport)
#  else
#    define SOTVISUALPOINTPROJECTER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTVISUALPOINTPROJECTER_EXPORT
#endif

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;
#include <sot/core/matrix-homogeneous.hh>

/* SOT */
#include <dynamic-graph/signal-helper.h>
#include <dynamic-graph/entity-helper.h>

namespace dynamicgraph {
  namespace sot {

    /* --------------------------------------------------------------------- */
    /* --- CLASS ----------------------------------------------------------- */
    /* --------------------------------------------------------------------- */


    class SOTVISUALPOINTPROJECTER_EXPORT VisualPointProjecter
      :public ::dynamicgraph::Entity
      ,public ::dynamicgraph::EntityHelper<VisualPointProjecter>
      {

      public: /* --- CONSTRUCTOR ---- */

	VisualPointProjecter( const std::string & name );

      public: /* --- ENTITY INHERITANCE --- */

	static const std::string CLASS_NAME;
	virtual void display( std::ostream& os ) const;
	virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

      public:  /* --- SIGNALS --- */

	DECLARE_SIGNAL_IN(point3D,ml::Vector);
	DECLARE_SIGNAL_IN(transfo,MatrixHomogeneous);

	DECLARE_SIGNAL_OUT(point3Dgaze,ml::Vector);
	DECLARE_SIGNAL_OUT(depth,double);
	DECLARE_SIGNAL_OUT(point2D,ml::Vector);

      }; // class VisualPointProjecter

  } // namespace sot
} // namespace dynamicgraph

#endif // #ifndef __sot_core_VisualPointProjecter_H__
