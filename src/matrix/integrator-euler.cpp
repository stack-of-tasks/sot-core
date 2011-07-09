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

#include <sot/core/integrator-euler.hh>

#include "integrator-abstract.t.cpp"
#include "integrator-euler.t.cpp"

#include <sot/core/integrator-euler-impl.hh>

#ifdef WIN32
  IntegratorEulerVectorMatrix::IntegratorEulerVectorMatrix( const std::string& name ) : 
	IntegratorEuler<Vector,Matrix>(name) {}
  std::string IntegratorEulerVectorMatrix::getTypeName( void ) { return "double"; }
#endif // WIN32

