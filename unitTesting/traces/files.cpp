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

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

using namespace std;

int main()
{
  
    boost::filesystem::create_directory( "foobar" );
    ofstream file( "foobar/cheeze" );
    file << "tastes good!\n";
    file.close();
    if ( !boost::filesystem::exists( "foobar/cheeze" ) )
      std::cout << "Something is rotten in foobar\n";



  return 0;
}
 
