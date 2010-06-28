/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Constraint.cpp
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


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot-core/constraint.h>
#include <sot-core/debug.h>
#include <dynamic-graph/pool.h>
using namespace std;
using namespace sot;

#include <sot-core/factory.h>
SOT_FACTORY_TASK_PLUGIN(Constraint,"Constraint");

using namespace dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


Constraint::
Constraint( const std::string& n )
  :TaskAbstract(n)
{
  jacobianSOUT.setFunction( boost::bind(&Constraint::computeJacobian,this,_1,_2) );
  
  signalDeregistration( "task" );
  signalDeregistration( "activation" );
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

namespace sot {
std::ostream& operator<< ( std::ostream& os,const Constraint& t )
{ return os << t.name; }
}


/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
void Constraint::
commandLine( const std::string& cmdLine
	     ,std::istringstream& cmdArgs
	     ,std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "Constraint: "<<endl
	 << "  - add <obj.signal>"<<endl
	 << "  - clear"<<endl;
      //TaskAbstract
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="add" )
    {
      SignalBase<int>* sigA = &g_pool.getSignal( cmdArgs );
      Signal< ml::Matrix,int >* sig 
	= dynamic_cast< Signal<ml::Matrix,int>* >( sigA );
      if(! sig ) 
	{
    	  if ( sigA )
			  SOT_THROW ExceptionSignal( ExceptionSignal::BAD_CAST,
							"Not a Matrix signal",
							": while casting signal <%s> (signal type is %s).",
							sigA->getName().c_str(), typeid(*sigA).name() );
    	  else
    		  SOT_THROW ExceptionSignal( ExceptionSignal::NOT_INITIALIZED,
    				  "Could not get a reference to requested signal");
	}
      addJacobian( *sig );
    }
  else if( cmdLine=="clear" )
    {
      clearJacobianList();
      jacobianSOUT.setReady();
    }
  else 
    TaskAbstract::commandLine( cmdLine,cmdArgs,os );

}
