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

#include <sot/core/contiifstream.hh>
#include <sot/core/debug.hh>

using namespace dynamicgraph::sot;

Contiifstream::
Contiifstream( const std::string& n )
  :filename(n),cursor(0),first(true) {}


Contiifstream::
~Contiifstream( void )
{
  sotDEBUGINOUT(5);
}


bool Contiifstream::
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
Contiifstream::next( void ) 
{
  std::string res = *reader.begin();
  reader.pop_front();
  return res;
}





