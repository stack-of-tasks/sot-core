/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Timer.cpp
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
#include <sot-core/timer.h>
#include <MatrixAbstractLayer/boost.h>
#include <sot-core/matrix-homogeneous.h>
#include <sot-core/factory.h>

using namespace sot;
using namespace dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

typedef Timer<maal::boost::Vector> timevect;
template <> 
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(timevect,"Timer<Vector>");

typedef Timer<maal::boost::Matrix> timematrix;
template <> 
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(timematrix,"Timer<Matrix>");

typedef Timer<MatrixHomogeneous> timematrixhomo;
template <> 
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(timematrixhomo,"Timer<MatrixHomo>");

typedef Timer<double> timedouble;
template <> 
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(timedouble,"Timer<double>");


/* --------------------------------------------------------------------- */
void 
cmdChrono( const std::string& cmdLine,
	   std::istringstream& cmdArg,
	   std::ostream& os )
{
  sotDEBUGIN(15);

  if( cmdLine == "help" ) 
    {
      os << "  - chrono <cmd...>"
	 << "\t\t\t\tLaunch <cmd> and display the time spent in the operation." <<std::endl;
      return;
    }
  
  struct timeval t0,t1;
  double dt;

  gettimeofday(&t0,NULL);
  sotDEBUG(15) << "t0: "<< t0.tv_sec << " - " << t0.tv_usec << std::endl;
  
  std::string cmdLine2;  cmdArg>>cmdLine2;
  sotDEBUG(5)<<"Chrono <" <<cmdLine2<<">"<<std::endl;
  g_shell.cmd(cmdLine2,cmdArg,os);

  gettimeofday(&t1,NULL);
  dt = ( (t1.tv_sec-t0.tv_sec) * 1000. 
	 + (t1.tv_usec-t0.tv_usec+0.) / 1000. );
  sotDEBUG(15) << "t1: "<< t1.tv_sec << " - " << t1.tv_usec << std::endl;

  os << "Time spent = " << dt << " ms " << std::endl;

  sotDEBUGOUT(15);
}



extern "C" {
  ShellFunctionRegisterer regTimer1
  ( "chrono",boost::bind(cmdChrono,_1,_2,_3) );
}


/* --------------------------------------------------------------------- */
/* --- CONTROL --------------------------------------------------------- */
/* --------------------------------------------------------------------- */



