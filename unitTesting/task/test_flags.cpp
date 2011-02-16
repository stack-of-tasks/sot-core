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

/* -------------------------------------------------------------------------- */
/* --- INCLUDES ------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#include <sot/core/flags.hh>
#include <iostream>
#include <sstream>

using namespace std;
using namespace dynamicgraph::sot;

int main( void )
{
  cout <<"Entering test" <<endl;
  Flags f1(128*112+84);
  Flags f2(198);
  cout<<"f1    "<<"\t"<<f1<<endl;
  cout<<"f2    "<<"\t"<<f2<<endl;

  cout<<endl;
  cout<<"1|2   "<<"\t"<<(f1|f2)<<endl;
  cout<<"1&2   "<<"\t"<<(f1&f2)<<endl;
  cout<<"TRUE  "<<"\t"<<(Flags(true))<<endl;
  cout<<"1&TRUE"<<"\t"<<(f1&Flags(true))<<endl;
  cout<<"1&!2 "<<"\t"<<(f1&!f2)<<endl;
  cout<<"1XOR2 "<<"\t"<<((f1&!f2)|(!f1&f2))<<endl;

  cout<<endl;
  cout<<"f1    "<<"\t"<<f1<<endl;
  cout<<"!2    "<<"\t"<<!f2<<endl;
  cout<<"1|!2 "<<"\t"<<(f1|!f2)<<endl;

  cout<<endl;
  if( f1&f2 ) cout<<"TRUE"; else cout<<"FALSE";  cout<<endl;
  if( f1&Flags() ) cout<<"TRUE"; else cout<<"FALSE";  cout<<endl;

  cout<<endl;
  cout<<"f1>>3 "<<"\t"<<Flags(f1>>3)<<endl;
  cout<<"f1>>5 "<<"\t"<<Flags(f1>>5)<<endl;

  cout<<"f1 byte per byte:";
  for(int i=0;i<16;++i) {if(! (i%8)) cout<<" ";  cout << f1(i); }  cout<<endl;

  cout<<endl;
  cout<<"L1   \t"<<FLAG_LINE_1<<endl;
  cout<<"L4   \t"<<FLAG_LINE_4<<endl;
  cout<<"L8   \t"<<FLAG_LINE_8<<endl;
  cout<<endl;

  istringstream iss("00101");
  Flags flread;
  iss >> flread;
  cout<<flread<<endl<<endl;


  return 0;
}
