/*
 * Copyright 2010,
 * Florent Lamiraux
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

#ifndef MATRIX_CONSTANT_COMMAND_H
 #define MATRIX_CONSTANT_COMMAND_H

 #include <boost/assign/list_of.hpp>

 #include <dynamic-graph/command.h>
 #include <dynamic-graph/command-setter.h>
 #include <dynamic-graph/command-getter.h>

namespace dynamicgraph {
  namespace sot {
    namespace command {
      namespace matrixConstant {
	using ::dynamicgraph::command::Command;
	using ::dynamicgraph::command::Value;
	
	// Command Resize
	class Resize : public Command
	{
	public:
	  virtual ~Resize() {}
	  /// Create command and store it in Entity
	  /// \param entity instance of Entity owning this command
	  /// \param docstring documentation of the command
	Resize(MatrixConstant& entity, const std::string& docstring) :
	  Command(entity, boost::assign::list_of(Value::UNSIGNED)
		  (Value::UNSIGNED), docstring)
	    {
	    }
	  virtual Value doExecute()
	  {
	    MatrixConstant& mc = static_cast<MatrixConstant&>(owner());
	    std::vector<Value> values = getParameterValues();
	    unsigned nbRows = values[0].value();
	    unsigned nbCols = values[1].value();
	    dynamicgraph::Matrix m(nbRows, nbCols);
	    m.fill(mc.color);
	    mc.SOUT.setConstant(m);
	    
	    // return void
	    return Value();
	  }
	}; // class Resize
      } // namespace matrixConstant
    } // namespace command
  } // namespace sot
} // namespace dynamicgraph

#endif //MATRIX_CONSTANT_COMMAND_H
