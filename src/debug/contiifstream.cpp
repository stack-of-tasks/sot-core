/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotContiifstream.cpp
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

#include <sot-core/sotContiifstream.h>
#include <sot-core/sotDebug.h>

sotContiifstream::
sotContiifstream( const std::string& n )
  :filename(n),cursor(0),first(true) {}


sotContiifstream::
~sotContiifstream( void )
{
  sotDEBUGINOUT(5);
}


bool sotContiifstream::
loop( void )
{
  sotDEBUGIN(25);
  bool res=false;
  
  std::fstream file( filename.c_str() );

  file.seekg(cursor);
  file.sync();
      
  while(1)
    {
      file.get(buffer,BUFFER_SIZE);
      if( file.gcount() ) 
	{ 
	  res=true;
	  std::string line(buffer);
	  if(! first) reader.push_back(line);
	  cursor=file.tellg(); cursor++;
	  file.get(*buffer); // get the last char ( = '\n')
	  sotDEBUG(15) << "line: "<< line<<std::endl;
	}
      else { break; }
    }

  first=false;
  sotDEBUGOUT(25);
  return res;
}

std::string 
sotContiifstream::next( void ) 
{
  std::string res = *reader.begin();
  reader.pop_front();
  return res;
}





