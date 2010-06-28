/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-JAPAN, Tsukuba, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_ptr.cc
 * Project:   Stack Of Tasks
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
#include <sot-core/debug.h>
#include <dynamic-graph/all-signals.h>
#include <iostream>

#include <MatrixAbstractLayer/boost.h>
#include <sot-core/vector-utheta.h>
#include <sot-core/exception-abstract.h>

using namespace std;
using namespace sot;
using namespace dynamicgraph;

namespace ml = maal::boost;




template< class Res=double >
class DummyClass
{

public:
  DummyClass( void ) : res(),appel(0),timedata(0) {}

  Res& fun( Res& res,int t) 
  {
    appel++;  timedata=t; 

    sotDEBUG(5) << "Inside " << typeid(Res).name() <<endl;
    for( list< SignalTimeDependent<double,int>* >::iterator it=inputsig.begin();
	 it!=inputsig.end();++it )
       { sotDEBUG(5) << *(*it) << endl; (*it)->access(timedata);}
    for( list< SignalTimeDependent<ml::Vector,int>* >::iterator it=inputsigV.begin();
	 it!=inputsigV.end();++it )
      { sotDEBUG(5) << *(*it) << endl; (*it)->access(timedata);}

    return res=(*this)();
  }

  list< SignalTimeDependent<double,int>* > inputsig;
  list< SignalTimeDependent<ml::Vector,int>* > inputsigV;

  void add( SignalTimeDependent<double,int>& sig ){ inputsig.push_back(&sig); }
  void add( SignalTimeDependent<ml::Vector,int>& sig ){ inputsigV.push_back(&sig); }

  Res operator() ( void );

  Res res;
  int appel;
  int timedata;
  
};

template< class Res >
Res DummyClass<Res>::operator() (void)
{ return this->res; }

template<>
double DummyClass<double>::operator() (void)
{
  res=appel*timedata; return res;
}
template<>
ml::Vector DummyClass<ml::Vector>::operator() (void)
{
  res.resize(3);
  res.fill(appel*timedata); return res;
}
template<>
VectorUTheta DummyClass<VectorUTheta>::operator() (void)
{
  res.fill(12.6); return res;
}


// void dispArray( const SignalArray<int> &ar )
// {
//   for( unsigned int i=0;i<ar.rank;++i ) sotDEBUG(5)<<*ar.array[i]<<endl;
// }

void funtest( ml::Vector& v ){ } 

#include <vector>
int main( void )
{
   DummyClass<VectorUTheta> pro3;

   SignalTimeDependent<VectorUTheta,int> sig3(sotNOSIGNAL,"Sig3");
   SignalPtr<ml::Vector,int> sigTo3( NULL,"SigTo3" );

   ml::Vector v;
   VectorUTheta v3;
   funtest(v);
   funtest(v3);

   sig3.setFunction( boost::bind(&DummyClass<VectorUTheta>::fun,pro3,_1,_2) );
   try 
     {
       sigTo3.plug(&sig3);
     }
   catch( sot::ExceptionAbstract& e ) { cout << e << endl; exit(1); }
   
   sig3.access(1); sig3.setReady();
   sigTo3.access(2);
   cout << sigTo3.access(2);

  return 0;

}
