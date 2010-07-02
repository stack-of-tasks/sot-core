/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-JAPAN, Tsukuba, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_chrono.cc
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

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

#include <sot-core/feature-abstract.h>
#include <sot-core/debug.h>


#include <iostream>
using namespace std;

#include <sot-core/sot.h>

#ifndef WIN32
#include <sys/time.h>
#else /*WIN32*/
#include <MatrixAbstractLayer/boost.h>
#include <Winsock2.h>
#include <sot/utils-windows.h>
#endif /*WIN32*/

#define sotCHRONO1 \
      gettimeofday(&t1,NULL);\
      dt = ( (t1.tv_sec-t0.tv_sec) * 1000.\
	     + (t1.tv_usec-t0.tv_usec+0.) / 1000. );\
      cout << "dt: "<< dt 


int main( int argc,char** argv )
{
  sotDEBUGIN(15);

  struct timeval t0,t1; double dt;
  
  ublas::matrix<double> P(40,40); 
  ublas::matrix<double> J(6,40);
  ublas::matrix<double> JK(6,40);
  for( unsigned int i=0;i<40;++i )
    for( unsigned int j=0;j<40;++j ) P(i,j) = (rand()+1.) / RAND_MAX;
  for( unsigned int i=0;i<J.size1();++i )
    for( unsigned int j=0;j<J.size2();++j ) J(i,j) = (rand()+1.) / RAND_MAX;
  
  int nbIter = 100000;
  dt=0;
  gettimeofday(&t0,NULL); 
  for( int iter=0;iter<nbIter;++iter )
    {
      gettimeofday(&t0,NULL); 
      //J.multiply(P,JK);
      //prod(J.matrix,P.matrix,JK.matrix);
      prod(J,P,JK);
      gettimeofday(&t1,NULL); 
      dt += ( (t1.tv_sec-t0.tv_sec) 
	     + (t1.tv_usec-t0.tv_usec+0.)  / 1000. / 1000. );
   
    }
  //sotCHRONO1 <<endl; 
  cout<<dt/nbIter<<endl;

  sotDEBUGOUT(15);
  return 0;
}




