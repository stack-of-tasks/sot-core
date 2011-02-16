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

#include <sot/core/debug.hh>
#include <fstream>
#include <ios>

using namespace dynamicgraph::sot;

namespace dynamicgraph { namespace sot {
#ifdef WIN32
const char * DebugTrace::DEBUG_FILENAME_DEFAULT = "c:/tmp/sot-core-traces.txt";
#else	// WIN32
const char * DebugTrace::DEBUG_FILENAME_DEFAULT = "/tmp/sot-core-traces.txt";
#endif	// WIN32

#ifdef VP_DEBUG
# ifdef WIN32
  std::ofstream debugfile("C:/tmp/sot-core-traces.txt",
			  std::ios::trunc&std::ios::out);
# else	// WIN32
  std::ofstream debugfile("/tmp/sot-core-traces.txt",
			  std::ios::trunc&std::ios::out);
# endif	// WIN32
#else // VP_DEBUG

  std::ofstream debugfile;

  class __sotDebug_init
  {
  public:
    __sotDebug_init()
    {
      debugfile.setstate (std::ios::failbit);
    }
  };
  __sotDebug_init __sotDebug_initialisator;

#endif // VP_DEBUG

} /* namespace sot */} /* namespace dynamicgraph */

void DebugTrace::openFile(const char * filename)
{
  if (debugfile.good () && debugfile.is_open ())
    debugfile.close ();
  debugfile.clear ();
  debugfile.open (filename, std::ios::trunc&std::ios::out);
}

void DebugTrace::closeFile(const char *)
{
  if (debugfile.good () && debugfile.is_open ())
    debugfile.close();
  debugfile.setstate (std::ios::failbit);
}

namespace dynamicgraph {
  namespace sot {
    DebugTrace sotDEBUGFLOW(debugfile);
    DebugTrace sotERRORFLOW(debugfile);
  } // namespace sot.
} // namespace dynamicgraph
