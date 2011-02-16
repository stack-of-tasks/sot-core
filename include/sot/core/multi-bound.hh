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

#ifndef __SOT_MultiBound_H__
#define __SOT_MultiBound_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* STD */
#include <string>
#include <vector>
#include <iostream>

/* SOT */
#include "sot/core/api.hh"
#include <sot/core/exception-task.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {

class SOT_CORE_EXPORT MultiBound
{
 public:
  enum MultiBoundModeType { MODE_SINGLE, MODE_DOUBLE };
  enum SupInfType { BOUND_SUP,BOUND_INF };

 public:// protected:
  MultiBoundModeType mode;
  double boundSingle;
  double boundSup,boundInf;
  bool boundSupSetup,boundInfSetup;

 public:
  MultiBound( const double x = 0.);
  MultiBound( const double xi,const double xs );
  MultiBound( const double x,const SupInfType bound );
  MultiBound( const MultiBound& clone );

 public: // Acessors
  MultiBoundModeType getMode( void ) const;
  double getSingleBound( void ) const;
  double getDoubleBound( const SupInfType bound ) const;
  bool getDoubleBoundSetup( const SupInfType bound ) const;

 public: // Modifiors
  void setDoubleBound( SupInfType boundType,double boundValue );
  void unsetDoubleBound( SupInfType boundType );
  void setSingleBound( double boundValue );

 public:
  SOT_CORE_EXPORT friend std::ostream& operator<< ( std::ostream& os, const MultiBound & m  );
  SOT_CORE_EXPORT friend std::istream& operator>> ( std::istream& is, MultiBound & m  );
};

/* --------------------------------------------------------------------- */
typedef std::vector< MultiBound > VectorMultiBound;
SOT_CORE_EXPORT std::ostream& operator<< (std::ostream& os, const VectorMultiBound& v );
SOT_CORE_EXPORT std::istream& operator>> (std::istream& os, VectorMultiBound& v );

} /* namespace sot */} /* namespace dynamicgraph */


#endif // #ifndef __SOT_MultiBound_H__
