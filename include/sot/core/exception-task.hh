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

#ifndef __SOT_EXCEPTION_TASK_H
#define __SOT_EXCEPTION_TASK_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot/core/exception-abstract.hh>
#include "sot/core/api.hh"
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {

/* \class ExceptionTask
 */
class SOT_CORE_EXPORT ExceptionTask
:public ExceptionAbstract

{
public:

  enum ErrorCodeEnum
    {
      GENERIC = ExceptionAbstract::TASK
      ,EMPTY_LIST
      ,NON_ADEQUATE_FEATURES
      ,MATRIX_SIZE
      ,BOUND_TYPE
      ,PARSER_MULTI_BOUND
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return EXCEPTION_NAME; }

  ExceptionTask ( const ExceptionTask::ErrorCodeEnum& errcode,
		     const std::string & msg = "" );
  ExceptionTask( const ExceptionTask::ErrorCodeEnum& errcode,
		    const std::string & msg,const char* format, ... );
  virtual ~ExceptionTask( void ) throw() {}

};

} /* namespace sot */} /* namespace dynamicgraph */



#endif /* #ifndef __SOT_EXCEPTION_TASK_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
