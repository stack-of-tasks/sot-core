/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_flags.cc
 * Project:   sot
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
//#undef WITH_OPENHRP

#include <iostream>
#include <sot-core/debug.h>

#include <MatrixAbstractLayer/boost.h>

#ifndef WIN32
#include <unistd.h>
#endif

using namespace std;
namespace ml = maal::boost;

#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>
#include <sot-core/feature-abstract.h>
#include <dynamic-graph/plugin-loader.h>
#include <dynamic-graph/interpreter.h>
#include <sot-core/mailbox.h>
#include <sstream>

using namespace dynamicgraph;
using namespace sot;

#ifdef  HAVE_LIBBOOST_THREAD
#include <boost/thread.hpp>

Mailbox<ml::Vector> mailbox("mail");

void f( void ) 
{ 
  ml::Vector vect(25);
  for( int i=0;;++i )
    {
      for( int j=0;j<25;++j ) vect(j) = j+i*10;
      mailbox.post( vect );
      //usleep( 1000*50 );
    }
}


int main( int argc,char** argv )
{
  boost::thread th( f );


  return 0;
}
#else
	int main()
	{
		cout << "This test cannot be run without LIBBOOST_THREAD" << endl;
		return 0;
	}
#endif
