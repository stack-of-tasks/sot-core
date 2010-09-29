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
#include <sot-core/task.h>
#include <sot-core/debug.h>
using namespace std;
using namespace sot;
using namespace dynamicgraph;


#include <sot-core/factory.h>
SOT_FACTORY_TASK_PLUGIN(Task,"Task");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


Task::
Task( const std::string& n )
  :TaskAbstract(n)
   ,featureList()
   ,controlGainSIN( NULL,"sotTask("+n+")::input(double)::controlGain" )
   ,dampingGainSINOUT( NULL,"sotTask("+n+")::in/output(double)::damping" )
   ,controlSelectionSIN( NULL,"sotTask("+n+")::input(flag)::controlSelec" )
   ,errorSOUT( boost::bind(&Task::computeError,this,_1,_2),
	       sotNOSIGNAL,
	       "sotTask("+n+")::output(vector)::error" )
{
  taskSOUT.setFunction( boost::bind(&Task::computeTaskExponentialDecrease,this,_1,_2) );
  jacobianSOUT.setFunction( boost::bind(&Task::computeJacobian,this,_1,_2) );
  featureActivationSOUT.setFunction( boost::bind(&Task::computeFeatureActivation,this,_1,_2) );

  taskSOUT.addDependency( controlGainSIN );
  taskSOUT.addDependency( errorSOUT );

  jacobianSOUT.addDependency( controlSelectionSIN );

  controlSelectionSIN = true;

  signalRegistration( controlGainSIN<<dampingGainSINOUT
		      <<controlSelectionSIN<<errorSOUT );
}



void Task::
addFeature( FeatureAbstract& s )
{
  featureList.push_back(&s);
  jacobianSOUT.addDependency( s.jacobianSOUT );
  errorSOUT.addDependency( s.errorSOUT );
  featureActivationSOUT.addDependency( s.activationSOUT );
}
void Task::
clearFeatureList( void )
{

  for(   std::list< FeatureAbstract* >::iterator iter = featureList.begin();
	 iter!=featureList.end(); ++iter )
    {
      FeatureAbstract & s = **iter;
      jacobianSOUT.removeDependency( s.jacobianSOUT );
      errorSOUT.removeDependency( s.errorSOUT );
      featureActivationSOUT.removeDependency( s.activationSOUT );
    }

  featureList.clear();
}

void Task::
setControlSelection( const Flags& act )
{
  controlSelectionSIN = act;
}
void Task::
addControlSelection( const Flags& act )
{
  Flags fl = controlSelectionSIN.accessCopy();
  fl &= act;
  controlSelectionSIN = fl;
}
void Task::
clearControlSelection( void )
{
  controlSelectionSIN = Flags(false);
}

/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */
/* --- COMPUTATION ---------------------------------------------------------- */

ml::Vector& Task::
computeError( ml::Vector& error,int time )
{
  sotDEBUG(15) << "# In " << getName() << " {" << endl;

  if( featureList.empty())
    { throw( ExceptionTask(ExceptionTask::EMPTY_LIST,
			      "Empty feature list") ) ; }

  try {
    /* The vector dimensions are not known before the affectation loop.
     * They thus should be allocated on the flight, in the loop.
     * The first assumption is that the size has not changed. A double
     * reallocation (realloc(dim*2)) is done if necessary. In particulary,
     * [log_2(dim)+1] reallocations are done for the first error computation.
     * If the allocated size is too large, a correction is done after the loop.
     * The algotithmic cost is linear in affectation, logarthmic in allocation
     * numbers and linear in allocation size.
     * No assumptions are made concerning size of each vector: they are
     * not said equal, and could be different.
     */

    /* First assumption: vector dimensions have not changed. If 0, they are
     * initialized to dim 1.*/
    int dimError = error .size();
    if( 0==dimError ){ dimError = 1; error.resize(dimError); }

    ml::Vector vectTmp;
    int cursorError = 0;

    /* For each cell of the list, recopy value of s, s_star and error. */
    for(   std::list< FeatureAbstract* >::iterator iter = featureList.begin();
	   iter!=featureList.end(); ++iter )
      {
	FeatureAbstract &feature = **iter;

	/* Get s, and store it in the s vector. */
	sotDEBUG(45) << "Feature <" << feature.getName() << ">." << std::endl;
	const ml::Vector& partialError = feature.errorSOUT(time);

	const int dim = partialError.size();
	while( cursorError+dim>dimError )  // DEBUG It was >=
	  { dimError *= 2; error.resize(dimError); }

	for( int k=0;k<dim;++k ){ error(cursorError++) = partialError(k); }
	sotDEBUG(35) << "feature: "<< partialError << std::endl;
	sotDEBUG(35) << "error: "<< error << std::endl;
      }

    /* If too much memory has been allocated, resize. */
    error .resize(cursorError,false);
  } catch SOT_RETHROW;

  sotDEBUG(35) << "error_final: "<< error << std::endl;
  sotDEBUG(15) << "# Out }" << endl;
  return error;
}

VectorMultiBound& Task::
computeTaskExponentialDecrease( VectorMultiBound& errorRef,int time )
{
  sotDEBUG(15) << "# In {" << endl;
  const ml::Vector & errSingleBound = errorSOUT(time);
  const double & gain = controlGainSIN(time);
  errorRef.resize( errSingleBound.size() );

  for( unsigned int i=0;i<errorRef.size(); ++i )
    errorRef[i] = - errSingleBound(i)*gain;

  sotDEBUG(15) << "# Out }" << endl;
  return errorRef;
}

ml::Matrix& Task::
computeJacobian( ml::Matrix& J,int time )
{
  sotDEBUG(15) << "# In {" << endl;

  if( featureList.empty())
    { throw( ExceptionTask(ExceptionTask::EMPTY_LIST,
			      "Empty feature list") ) ; }

  try {
    unsigned int dimJ = J .nbRows();
    unsigned int nbc = J.nbCols();
    if( 0==dimJ ){ dimJ = 1; J.resize(dimJ,nbc); }

    int cursorJ = 0;
    //const Flags& selection = controlSelectionSIN(time);

    /* For each cell of the list, recopy value of s, s_star and error. */
    for(   std::list< FeatureAbstract* >::iterator iter = featureList.begin();
	   iter!=featureList.end(); ++iter )
      {
	FeatureAbstract &feature = ** iter;
	sotDEBUG(25) << "Feature <" << feature.getName() <<">"<< endl;

	/* Get s, and store it in the s vector. */
	const ml::Matrix& partialJacobian = feature.jacobianSOUT(time);
	const unsigned int nbr = partialJacobian.nbRows();
	sotDEBUG(25) << "Jp =" <<endl<< partialJacobian<<endl;

	if( 0==nbc ) { nbc = partialJacobian.nbCols(); J.resize(nbc,dimJ); }
	else if( partialJacobian.nbCols() != nbc )
	  throw ExceptionTask(ExceptionTask::NON_ADEQUATE_FEATURES,
				 "Features from the list don't have compatible-size jacobians.");

	while( cursorJ+nbr>=dimJ )
	  { dimJ *= 2; J.resize(dimJ,nbc,false); }
	for( unsigned int kc=0;kc<nbc;++kc )
	  {
	    // 	  if( selection(kc) )
	    for( unsigned int k=0;k<nbr;++k )
	      { J(cursorJ+k,kc) = partialJacobian(k,kc); }
	    // 	  else
	    // 	    for( unsigned int k=0;k<nbr;++k ) J(cursorJ+k,kc) = 0.;
	  }
	cursorJ += nbr;
      }

    /* If too much memory has been allocated, resize. */
    J .resize(cursorJ,nbc,false);
  } catch SOT_RETHROW;


  sotDEBUG(15) << "# Out }" << endl;
  return J;
}



ml::Vector& Task::
computeFeatureActivation( ml::Vector& activation,int time )
{
  sotDEBUG(15) << "# In {" << endl;
  if( featureList.empty())
    { throw( ExceptionTask(ExceptionTask::EMPTY_LIST,
			      "Empty feature list") ) ; }

  try {
    int dimH = activation.size();
    if( 0==dimH ){ dimH = 1; activation.resize(dimH); }

    int cursorH = 0;

    /* For each cell of the list, recopy value of s, s_star and error. */
    for(   std::list< FeatureAbstract* >::iterator iter = featureList.begin();
	   iter!=featureList.end(); ++iter )
      {
	FeatureAbstract &feature = ** iter;

	/* Get s, and store it in the s vector. */
	const ml::Vector& partialActivation = feature.activationSOUT(time);
	const int nbr = partialActivation.size();
	sotDEBUG(25) << "hp =" << partialActivation<<endl;

	while( cursorH+nbr>=dimH )
	  { dimH *= 2; activation.resize(dimH,false); }
	for( int k=0;k<nbr;++k,++cursorH )
	  { activation(cursorH) = partialActivation(k); }
      }

    /* If too much memory has been allocated, resize. */
    activation.resize(cursorH,false);
  } catch SOT_RETHROW;


  sotDEBUG(15) << "# Out }" << endl;
  return activation;
}

/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

void Task::
display( std::ostream& os ) const
{
  os << "Task " << name << ": " << endl;
  os << "--- LIST ---  " << std::endl;

  for(   std::list< FeatureAbstract* >::const_iterator iter = featureList.begin();
	 iter!=featureList.end(); ++iter )
    {
      os << "-> " << (*iter)->getName() <<endl;
    }

}



/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
#include <sot-core/pool.h>

static void readListIdx( std::istringstream& cmdArgs,
			 unsigned int & idx_beg,unsigned int &idx_end,
			 bool& no_end )
{
  char col;

  cmdArgs >> ws;
  if(! cmdArgs.good()) { idx_end=idx_beg=0; no_end=false; return; }
  cmdArgs.get(col); if( col==':' )
    { idx_beg=0; cmdArgs>>ws;}
  else
    {
      cmdArgs.putback(col); cmdArgs>>idx_beg>>ws;
      cmdArgs.get(col);
      if( col!=':' ) { idx_end=idx_beg; no_end=false; return; }
    }
  cmdArgs>>ws;
  if( cmdArgs.good() )
    {
      sotDEBUG(15) << "Read end" << endl;
      cmdArgs >> idx_end; no_end=false;
    }
  else no_end = true;

  sotDEBUG(25) <<"Selec: " << idx_beg << " : "<< idx_end
	       << "(" << no_end <<")"<<endl;
}

void Task::
commandLine( const std::string& cmdLine
	     ,std::istringstream& cmdArgs
	     ,std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "Task: "<<endl
	 << "  - add <feature>"<<endl
	 << "  - [un]selec [init] : [end] :"<<endl
	 << "modify the default value of the controlSelec signal"<<endl
	 << "  - clear"<<endl;
      TaskAbstract::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="add" )
    {
      std::string f; cmdArgs >> f;
      FeatureAbstract& feat = sotPool.getFeature( f );
      addFeature( feat );
    }
  else if( cmdLine=="selec" )
    {
      unsigned int idx_beg,idx_end;
      bool base;
      readListIdx( cmdArgs,idx_beg,idx_end,base );

      Flags newFlag( base );
      if(base)
	{
	  for( unsigned int i=0;i<idx_beg;++i)
	    newFlag .unset(i);
	}
      else for( unsigned int i=idx_beg;i<=idx_end;++i) newFlag.set(i);

      sotDEBUG(15) << "Next flag: "<<newFlag<<endl;
      sotDEBUG(15) << "Old flag: "<<controlSelectionSIN.accessCopy()<<endl;

      newFlag |= controlSelectionSIN.accessCopy();
      controlSelectionSIN = newFlag;

      sotDEBUG(15) << "New flag: "<<newFlag<<endl;

    }
  else if( cmdLine=="unselec" )
    {
      unsigned int idx_beg,idx_end; bool base;
      readListIdx( cmdArgs,idx_beg,idx_end,base );

      Flags newFlag(! base );
      if(base)
	{
	  for( unsigned int i=0;i<idx_beg;++i)
	    newFlag .set(i);
	}
      else for( unsigned int i=idx_beg;i<=idx_end;++i) newFlag.unset(i);

      sotDEBUG(15) << "Next flag: "<<newFlag<<endl;
      sotDEBUG(15) << "Old flag: "<<controlSelectionSIN.accessCopy()<<endl;

      newFlag &= controlSelectionSIN.accessCopy();
      controlSelectionSIN = newFlag;

      sotDEBUG(15) << "New flag: "<<newFlag<<endl;
    }
  else if( cmdLine=="unselec" )
    {
      unsigned int idx_beg,idx_end; bool base;
      readListIdx( cmdArgs,idx_beg,idx_end,base );

      Flags newFlag(! base );
      if(base)
	{
	  for( unsigned int i=0;i<idx_beg;++i)
	    newFlag .set(i);
	}
      else for( unsigned int i=idx_beg;i<=idx_end;++i) newFlag.unset(i);

      sotDEBUG(15) << "Next flag: "<<newFlag<<endl;
      sotDEBUG(15) << "Old flag: "<<controlSelectionSIN.accessCopy()<<endl;

      newFlag &= controlSelectionSIN.accessCopy();
      controlSelectionSIN = newFlag;

      sotDEBUG(15) << "New flag: "<<newFlag<<endl;
    }
  else if( cmdLine=="clear" )
    {
      clearFeatureList();
    }
  else  { TaskAbstract::commandLine( cmdLine,cmdArgs,os ); }

}
std::ostream & Task::
writeGraph(std::ostream &os) const
{
  std::list< FeatureAbstract * >::const_iterator itFeatureAbstract;
  itFeatureAbstract = featureList.begin();
  while(itFeatureAbstract!=featureList.end())
    {
      os << "\t\"" << (*itFeatureAbstract)->getName()  << "\" -> \"" << getName() << "\"" << endl;
      itFeatureAbstract++;
    }
  return os;
}
