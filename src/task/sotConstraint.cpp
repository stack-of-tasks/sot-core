/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotConstraint.cpp
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
#include <sot-core/sotConstraint.h>
#include <sot-core/sotDebug.h>
using namespace std;



#include <sot-core/factory.h>
SOT_FACTORY_TASK_PLUGIN(sotConstraint,"Constraint");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


sotConstraint::
sotConstraint( const std::string& n )
  :sotTaskAbstract(n)
{
  jacobianSOUT.setFunction( boost::bind(&sotConstraint::computeJacobian,this,_1,_2) );
  
  signalDeregistration( "task" );
  signalDeregistration( "activation" );
}



void sotConstraint::
addJacobian( Signal< ml::Matrix,int >& sig )
{
  sotDEBUGIN(15);
  jacobianList.push_back(&sig);
  jacobianSOUT.addDependancy( sig );
  sotDEBUGOUT(15);
}
void sotConstraint::
clearJacobianList( void )
{

  for(   JacobianList::iterator iter = jacobianList.begin();
	 iter!=jacobianList.end(); ++iter )
    {
      Signal< ml::Matrix,int >& s = **iter;
      jacobianSOUT.removeDependancy( s );
    }

  jacobianList.clear();
}

/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

ml::Matrix& sotConstraint::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15) << "# In {" << endl;

  if( jacobianList.empty())
    { J.resize(0,0); }
//    { throw( sotExceptionTask(sotExceptionTask::EMPTY_LIST,
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
	  {SOT_THROW sotExceptionTask(sotExceptionTask::NON_ADEQUATE_FEATURES,
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

std::ostream& operator<< ( std::ostream& os,const sotConstraint& t )
{ return os << t.name; }


/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
#include <dynamic-graph/pool.h>

void sotConstraint::
commandLine( const std::string& cmdLine
	     ,std::istringstream& cmdArgs
	     ,std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "Constraint: "<<endl
	 << "  - add <obj.signal>"<<endl
	 << "  - clear"<<endl;
      //sotTaskAbstract
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="add" )
    {
      SignalBase<int>* sigA = &pool.getSignal( cmdArgs );
      Signal< ml::Matrix,int >* sig 
	= dynamic_cast< Signal<ml::Matrix,int>* >( sigA );
      if(! sig ) 
	{
    	  if ( sigA )
			  SOT_THROW sotExceptionSignal( sotExceptionSignal::BAD_CAST,
							"Not a Matrix signal",
							": while casting signal <%s> (signal type is %s).",
							sigA->getName().c_str(), typeid(*sigA).name() );
    	  else
    		  SOT_THROW sotExceptionSignal( sotExceptionSignal::NOT_INITIALIZED,
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
    sotTaskAbstract::commandLine( cmdLine,cmdArgs,os );

}
