/*
 *  Copyright
 */

#include <iostream>
using namespace std;
#include <sot-core/debug.h>

int main (int argc, char** argv)
{
	sot::sotDEBUGFLOW.openFile();
	sot::sotDEBUGFLOW.trace("test test test");
	//sot::sotDEBUGFLOW << "whatwhatwhat" << std::endl;;
  cout << "It works!" << endl;
}
