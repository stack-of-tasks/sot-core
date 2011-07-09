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

#include <sot/core/integrator-abstract.hh>

// The specilization
#include "integrator-abstract.t.cpp"

// This ends the specialization part.
// Note that on WIN32, the specialization has to be realized 
//  before the declaration of the general model. 
#include <sot/core/integrator-abstract-impl.hh>

#ifdef WIN32
  IntegratorAbstractDouble::IntegratorAbstractDouble( const std::string& name ) : 
		IntegratorAbstract<double,double> (name) {}

  IntegratorAbstractVector::IntegratorAbstractVector( const std::string& name ) : 
		IntegratorAbstract<ml::Vector,ml::Matrix> (name) {}
#endif

