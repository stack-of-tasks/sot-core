/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotAdditionalFunctions.h
 * Project:   SOT
 * Author:    François Bleibel
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

/* SOT */
#include <dynamic-graph/signal-base.h>
#include <sot-core/exception-factory.h>
#include <dynamic-graph/pool.h>
#include <sot-core/sotPool.h>
#include <sot-core/sot-core-api.h>

/* --- STD --- */
#include <string>
#include <map>
#include <sstream>

/* --- BOOST --- */
#include <boost/function.hpp>
#include <boost/bind.hpp>

/*! @ingroup factory
  \brief This helper class dynamically overloads the "new" shell command
  to allow creation of tasks and features as well as entities.
 */
class sotAdditionalFunctions
{
public:
	sotAdditionalFunctions();
	~sotAdditionalFunctions();
	static void cmdNew( const std::string& cmd,std::istringstream& args,
						   std::ostream& os );
	static void cmdMatrixDisplay( const std::string& cmd,std::istringstream& args,
						   std::ostream& os );
	static void cmdFlagSet( const std::string& cmd,std::istringstream& args,
						   std::ostream& os );
};
