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

/* SOT */
#include <dynamic-graph/signal-base.h>
#include <sot-core/exception-factory.h>
#include <dynamic-graph/pool.h>
#include <sot-core/pool.h>
#include <sot-core/sot-core-api.h>

/* --- STD --- */
#include <string>
#include <map>
#include <sstream>

/* --- BOOST --- */
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace sot {

/*! @ingroup factory
  \brief This helper class dynamically overloads the "new" shell command
  to allow creation of tasks and features as well as entities.
 */
class AdditionalFunctions
{
public:
	AdditionalFunctions();
	~AdditionalFunctions();
	static void cmdNew( const std::string& cmd,std::istringstream& args,
						   std::ostream& os );
	static void cmdMatrixDisplay( const std::string& cmd,std::istringstream& args,
						   std::ostream& os );
	static void cmdFlagSet( const std::string& cmd,std::istringstream& args,
						   std::ostream& os );
};

} // namespace sot
