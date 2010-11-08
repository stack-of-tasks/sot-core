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

#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>

/* Matrix */
#include <jrl/mal/boost.hh>
namespace ml = maal::boost;



/* --------------------------------------------------------------------- */
/* --- MATRIX ---------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {
namespace dg = dynamicgraph;

class MatrixConstant
: public dg::Entity
{
 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

  int rows,cols;
  double color;

public:
  MatrixConstant( const std::string& name )
    :Entity( name )
    ,rows(0),cols(0),color(0.)
    ,SOUT( "sotMatrixConstant("+name+")::output(matrix)::out" )
    {
      SOUT.setDependencyType( dg::TimeDependency<int>::BOOL_DEPENDENT );
      signalRegistration( SOUT );
    }

  virtual ~MatrixConstant( void ){}

  dg::SignalTimeDependent<ml::Matrix,int> SOUT;

  virtual void commandLine( const std::string& cmdLine,
			    std::istringstream& cmdArgs, 
			    std::ostream& os );

};
    
} // namespace sot





