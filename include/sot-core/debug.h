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

#ifndef __VS_DEBUG_HH
#define __VS_DEBUG_HH

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdarg.h>
#include <sot-core/sot-core-api.h>

namespace sot {


/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

#ifndef VP_DEBUG_MODE
#define VP_DEBUG_MODE 0
#endif
#ifndef VP_TEMPLATE_DEBUG_MODE
#define VP_TEMPLATE_DEBUG_MODE 0
#endif

#define SOT_COMMON_TRACES do {  \
		    va_list arg; \
		    va_start(arg,format); \
		    vsnprintf( charbuffer,SIZE,format,arg ); \
		    va_end(arg); \
		    outputbuffer << tmpbuffer.str() << charbuffer <<std::endl; \
		} while(0)

class SOT_CORE_EXPORT DebugTrace
{
 public:
    static const int SIZE = 512;

    std::stringstream tmpbuffer;
    std::ostream& outputbuffer;
    char charbuffer[SIZE+1];
    int traceLevel;
    int traceLevelTemplate;

    DebugTrace( std::ostream& os ): outputbuffer(os) {}

    inline void trace( const int level,const char* format,...)
	{ if( level<=traceLevel ) SOT_COMMON_TRACES; tmpbuffer.str(""); }
    inline void trace( const char* format,...){ SOT_COMMON_TRACES;  tmpbuffer.str(""); }
    inline void trace( const int level=-1 )
	{ if( level<=traceLevel ) outputbuffer << tmpbuffer.str(); tmpbuffer.str("");  }

    inline void traceTemplate( const int level,const char* format,...)
	{ if( level<=traceLevelTemplate ) SOT_COMMON_TRACES; tmpbuffer.str(""); }
    inline void traceTemplate( const char* format,...)
	{ SOT_COMMON_TRACES; tmpbuffer.str("");  }

    inline DebugTrace& pre( const std::ostream& dummy ) { return *this; }
    inline DebugTrace& pre( const std::ostream& dummy,int level )
	{ traceLevel = level; return *this; }
/*     inline DebugTrace& preTemplate( const std::ostream& dummy,int level )  */
/* 	{ traceLevelTemplate = level; return *this; } */


    static const char * DEBUG_FILENAME_DEFAULT;
    static void openFile( const char * filename = DEBUG_FILENAME_DEFAULT );
    static void closeFile( const char * filename = DEBUG_FILENAME_DEFAULT );

};

SOT_CORE_EXPORT extern DebugTrace sotDEBUGFLOW;
SOT_CORE_EXPORT extern DebugTrace sotERRORFLOW;

#ifdef VP_DEBUG
#define sotPREDEBUG  __FILE__ << ": " <<__FUNCTION__  \
                              << "(#" << __LINE__ << ") :"
#define sotPREERROR  "\t!! "<<__FILE__ << ": " <<__FUNCTION__  \
                            << "(#" << __LINE__ << ") :"

#  define sotDEBUG(level) if( (level>VP_DEBUG_MODE)||(!sot::sotDEBUGFLOW.outputbuffer.good()) ) ;\
    else sot::sotDEBUGFLOW.outputbuffer << sotPREDEBUG
#  define sotDEBUGMUTE(level) if( (level>VP_DEBUG_MODE)||(!sot::sotDEBUGFLOW.outputbuffer.good()) ) ;\
    else sot::sotDEBUGFLOW.outputbuffer
#  define sotERROR  if(!sot::sotDEBUGFLOW.outputbuffer.good()); else sot::sotERRORFLOW.outputbuffer << sotPREERROR
#  define sotDEBUGF if(!sot::sotDEBUGFLOW.outputbuffer.good()); else sot::sotDEBUGFLOW.pre(sot::sotDEBUGFLOW.tmpbuffer<<sotPREDEBUG,VP_DEBUG_MODE).trace
#  define sotERRORF if(!sot::sotDEBUGFLOW.outputbuffer.good()); else sot::sotERRORFLOW.pre(sot::sotERRORFLOW.tmpbuffer<<sotPREERROR).trace
// TEMPLATE
#  define sotTDEBUG(level) if( (level>VP_TEMPLATE_DEBUG_MODE)||(!sot::sotDEBUGFLOW.outputbuffer.good()) ) ;\
    else sot::sotDEBUGFLOW.outputbuffer << sotPREDEBUG
#  define sotTDEBUGF  if(!sot::sotDEBUGFLOW.outputbuffer.good()); else sot::sotDEBUGFLOW.pre(sot::sotDEBUGFLOW.tmpbuffer<<sotPREDEBUG,VP_TEMPLATE_DEBUG_MODE).trace
inline bool sotDEBUG_ENABLE( const int & level ) { return level<=VP_DEBUG_MODE; }
inline bool sotTDEBUG_ENABLE( const int & level ) { return level<=VP_TEMPLATE_DEBUG_MODE; }

/* -------------------------------------------------------------------------- */
#else // #ifdef VP_DEBUG
#define sotPREERROR  "\t!! "<<__FILE__ << ": " <<__FUNCTION__  \
                            << "(#" << __LINE__ << ") :"
#  define sotDEBUG(level) if( 1 ) ; else std::cout
#  define sotDEBUGMUTE(level) if( 1 ) ; else std::cout
#  define sotERROR sotERRORFLOW.outputbuffer << sotPREERROR
inline void sotDEBUGF( const int level,const char* format,...) { return; }
inline void sotDEBUGF( const char* format,...) { return; }
inline void sotERRORF( const int level,const char* format,...) { return; }
inline void sotERRORF( const char* format,...) { return; }
// TEMPLATE
#  define sotTDEBUG(level) if( 1 ) ; else std::cout
inline void sotTDEBUGF( const int level,const char* format,...) { return; }
inline void sotTDEBUGF( const char* format,...) { return; }
#define sotDEBUG_ENABLE(level) false
#define sotTDEBUG_ENABLE(level) false

#endif // #ifdef VP_DEBUG
/* -------------------------------------------------------------------------- */

#define sotDEBUGIN(level) sotDEBUG(level) << "# In {" << std::endl
#define sotDEBUGOUT(level) sotDEBUG(level) << "# Out }" << std::endl
#define sotDEBUGINOUT(level) sotDEBUG(level) << "# In/Out { }" << std::endl

#define sotTDEBUGIN(level) sotTDEBUG(level) << "# In {" << std::endl
#define sotTDEBUGOUT(level) sotTDEBUG(level) << "# Out }" << std::endl
#define sotTDEBUGINOUT(level) sotTDEBUG(level) << "# In/Out { }" << std::endl

} // namespace sot

#endif /* #ifdef __VS_DEBUG_HH */

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
