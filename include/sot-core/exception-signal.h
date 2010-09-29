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

#ifndef __SOT_SIGNAL_EXCEPTION_H
#define __SOT_SIGNAL_EXCEPTION_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/exception-abstract.h>
#include <sot-core/sot-core-api.h>
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace sot {

/* \class ExceptionSignal
 */
class SOT_CORE_EXPORT ExceptionSignal
:public ExceptionAbstract

{
 public:
  enum ErrorCodeEnum
    {
      GENERIC = ExceptionAbstract::SIGNAL

      ,READWRITE_LOCK
      ,COPY_NOT_INITIALIZED
      ,NOT_INITIALIZED
      ,PLUG_IMPOSSIBLE
      ,SET_IMPOSSIBLE
      ,BAD_CAST
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return EXCEPTION_NAME; }

public:

  ExceptionSignal ( const ExceptionSignal::ErrorCodeEnum& errcode,
		       const std::string & msg = "" );
  ExceptionSignal( const ExceptionSignal::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );
  virtual ~ExceptionSignal( void ) throw() {}


};


} // namespace sot


#endif /* #ifndef __SOT_SIGNAL_EXCEPTION_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
