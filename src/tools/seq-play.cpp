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
#include <sot-core/seq-play.h>
#include <sot-core/debug.h>
using namespace std;

#include <fstream>
#include <sstream>

#include <dynamic-graph/factory.h>
using namespace dynamicgraph;
using namespace sot;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SeqPlay,"SeqPlay");


/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */


SeqPlay::
SeqPlay( const std::string& n )
  :Entity(n)
   ,stateList()
   ,currPos(stateList.begin())
   ,currRank(0)
   ,init(false)
   ,time(0)
   ,refresherSINTERN( "SeqPlay("+n+")::intern(dummy)::refresher" )
   ,positionSOUT( boost::bind(&SeqPlay::getNextPosition,this,_1,_2),
		  refresherSINTERN,
		  "SeqPlay("+n+")::output(vector)::position" )
{
  signalRegistration( positionSOUT );
  refresherSINTERN.setDependencyType( TimeDependency<int>::ALWAYS_READY );
}

/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */
/* --- COMPUTE ----------------------------------------------------------- */
ml::Vector& SeqPlay::
getNextPosition( ml::Vector& pos, const int& time )
{
  sotDEBUGIN(15);
  if( !init ) 
    {
      if( stateList.empty() ) return pos; 
      currPos=stateList.begin(); init=true; currRank = 0;
    }

  
    {
      const ml::Vector& posCur = *currPos;
      pos=posCur;
      
      currPos++; 
      if( currPos==stateList.end() ) currPos--;
      else currRank++;
    } 

  sotDEBUGOUT(15);
  return pos;
}

/* --- LIST -------------------------------------------------------------- */
/* --- LIST -------------------------------------------------------------- */
/* --- LIST -------------------------------------------------------------- */
void SeqPlay::
loadFile( const std::string& filename )
{
  sotDEBUGIN(15);

  sotDEBUG( 25 ) << " Load " << filename << endl;
  std::ifstream file(filename.c_str());
  const unsigned int SIZE = 1024;
  char buffer[SIZE];

  ml::Vector res(1); unsigned int ressize = 1;
  double time;

  while( file.good() )
    {
      file.getline( buffer,SIZE );
      if( file.gcount()<5 ) break; 

      sotDEBUG(25) << buffer<<endl;
      std::istringstream iss( buffer );
      
      iss>>time;      unsigned int i;

      for( i=0;iss.good();++i )
	{
	  if( i==ressize ) { ressize*=2; res.resize(ressize,false); }
	  iss>>res(i); sotDEBUG(35) <<i<< ": " <<  res(i)<<endl;
	}
      ressize=i-1;  res.resize(ressize,false); 
      stateList.push_back( res );
      sotDEBUG(15) << time << ": " <<  res << endl;
    }

  sotDEBUGOUT(15);
}




/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */
/* --- DISPLAY ------------------------------------------------------------ */

void SeqPlay::display ( std::ostream& os ) const
{os <<name<<endl; }


/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
/* --- PARAMS --------------------------------------------------------------- */
#include <dynamic-graph/pool.h>

void SeqPlay::
commandLine( const std::string& cmdLine
	     ,std::istringstream& cmdArgs
	     ,std::ostream& os )
{
  if( cmdLine=="help" )
    {
      os << "SeqPlay: "<<endl
	 << "  - load <file>"<<endl
	 << "  - size" <<endl;
	Entity::commandLine( cmdLine,cmdArgs,os );
    }
  else if( cmdLine=="load" )
    {
      std::string n; cmdArgs >> n;
      loadFile(n);
    }
  else if( cmdLine == "empty" )
    {
      stateList.clear(); init=false;
    }
  else if( cmdLine == "size" ) 
    {
      os << "size = " << stateList.size() << endl; 
      os << "rank = " << currRank << endl; 
    }
  else  
    {
      Entity::commandLine( cmdLine,cmdArgs,os );
    }
}
