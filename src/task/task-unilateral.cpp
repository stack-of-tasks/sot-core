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

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

//#define VP_DEBUG
//#define VP_DEBUG_MODE 15

/* SOT */
#include <sot/core/task-unilateral.hh>
#include <sot/core/debug.hh>

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;


#include <sot/core/factory.hh>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskUnilateral,"TaskUnilateral");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


TaskUnilateral::
TaskUnilateral( const std::string& n )
  :Task(n)
  ,featureList()
  ,positionSIN( NULL,"sotTaskUnilateral("+n+")::input(vector)::position" )
  ,referenceInfSIN( NULL,"sotTaskUnilateral("+n+")::input(vector)::referenceInf" )
  ,referenceSupSIN( NULL,"sotTaskUnilateral("+n+")::input(vector)::referenceSup" )
  ,dtSIN( NULL,"sotTaskUnilateral("+n+")::input(double)::dt" )
{
  taskSOUT.setFunction( boost::bind(&TaskUnilateral::computeTaskUnilateral,this,_1,_2) );
  taskSOUT.clearDependencies();
  taskSOUT.addDependency( referenceSupSIN );
  taskSOUT.addDependency( referenceInfSIN );
  taskSOUT.addDependency( dtSIN );
  taskSOUT.addDependency( positionSIN );

  signalRegistration( referenceSupSIN<<dtSIN<<referenceInfSIN<<positionSIN );
}


/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

VectorMultiBound& TaskUnilateral::
computeTaskUnilateral( VectorMultiBound& res,int time )
{
  sotDEBUG(45) << "# In " << getName() << " {" << endl;
  const ml::Vector & position = positionSIN(time);
  sotDEBUG(35) << "position = " << position << endl;
  const ml::Vector & refInf = referenceInfSIN(time);
  const ml::Vector & refSup = referenceSupSIN(time);
  const double & dt = dtSIN(time);
  res.resize(position.size());
  for( unsigned int i=0;i<res.size();++i )
    {
      MultiBound toto((refInf(i)-position(i))/dt,(refSup(i)-position(i))/dt);
      res[i] = toto;
    }

  sotDEBUG(15) << "taskU = "<< res << std::endl;
  sotDEBUG(45) << "# Out }" << endl;
  return res;
}

/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

void TaskUnilateral::
display( std::ostream& os ) const
{
  os << "TaskUnilateral " << name << ": " << endl;
}


