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

/* SOT */
#include <sot/core/constraint.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/pool.h>
using namespace std;
using namespace dynamicgraph::sot;

#include <sot/core/factory.hh>

using namespace dynamicgraph;
#include "../src/task/constraint-command.h"
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Constraint,"Constraint");

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


Constraint::
Constraint( const std::string& n )
  :TaskAbstract(n)
{
  jacobianSOUT.setFunction( boost::bind(&Constraint::computeJacobian,this,_1,_2) );
  
  signalDeregistration( taskSOUT.shortName() );

  //
  // Commands
  //
  std::string docstring;
  // Addjacobian
  docstring = "    \n"
    "    \n"
    "    Add a Jacobian to the constraint\n"
    "    \n"
    "      Input:\n"
    "        - name of the signal conveying the Jacobian.\n"
    "    \n";
  addCommand("addJacobian",
	     new command::constraint::AddJacobian(*this, docstring));
}



void Constraint::
addJacobian( Signal< ml::Matrix,int >& sig )
{
  sotDEBUGIN(15);
  jacobianList.push_back(&sig);
  jacobianSOUT.addDependency( sig );
  sotDEBUGOUT(15);
}
void Constraint::
clearJacobianList( void )
{

  for(   JacobianList::iterator iter = jacobianList.begin();
	 iter!=jacobianList.end(); ++iter )
    {
      Signal< ml::Matrix,int >& s = **iter;
      jacobianSOUT.removeDependency( s );
    }

  jacobianList.clear();
}

/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

ml::Matrix& Constraint::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15) << "# In {" << endl;

  if( jacobianList.empty())
    { J.resize(0,0); }
  //    { throw( ExceptionTask(ExceptionTask::EMPTY_LIST,
  // 			      "Empty feature list") ) ; }

  try {
    unsigned int dimJ = J .nbRows();
    unsigned int nbc = J.nbCols();
    if( 0==dimJ ){ dimJ = 1; J.resize(dimJ,nbc); }

    int cursorJ = 0;

    /* For each cell of the list, recopy value of s, s_star and error. */
    for( JacobianList::iterator iter = jacobianList.begin();
	 iter!=jacobianList.end(); ++iter )
      {
	Signal< ml::Matrix,int >& jacobian = ** iter;

	/* Get s, and store it in the s vector. */
	const ml::Matrix& partialJacobian = jacobian(time);
	const unsigned int nbr = partialJacobian.nbRows();
	
	if( 0==nbc ) { nbc = partialJacobian.nbCols(); J.resize(nbc,dimJ); }
	else if( partialJacobian.nbCols() != nbc )
	  {SOT_THROW ExceptionTask(ExceptionTask::NON_ADEQUATE_FEATURES,
				   "Features from the list don't "
				   "have compatible-size jacobians.");}
	sotDEBUG(25) << "Jp =" <<endl<< partialJacobian<<endl;

	while( cursorJ+nbr>=dimJ ) 
	  { dimJ *= 2; J.resize(dimJ,nbc,false); }
	for( unsigned int kc=0;kc<nbc;++kc ) 
	  for( unsigned int k=0;k<nbr;++k )
	    { J(cursorJ+k,kc) = partialJacobian(k,kc); }
	cursorJ += nbr;
      }
    
    /* If too much memory has been allocated, resize. */
    J .resize(cursorJ,nbc,false);
  } catch SOT_RETHROW;

  sotDEBUG(15) << "# Out }" << endl;
  return J; 
}




/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

namespace dynamicgraph { 
  namespace sot {
    std::ostream& operator<< ( std::ostream& os,const Constraint& t )
    { return os << t.name; }
  } // namespace sot
} // namespace dynamicgraph


