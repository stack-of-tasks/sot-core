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

#ifndef __SOT_EXCEPTION_FEATURE_H
#define __SOT_EXCEPTION_FEATURE_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#include <sot-core/exception-abstract.h>
#include "sot/core/api.hh"
/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph { namespace sot {

/* \class ExceptionFeature
 */
class SOT_CORE_EXPORT ExceptionFeature
:public ExceptionAbstract

{
public:

  enum ErrorCodeEnum
    {
      GENERIC = ExceptionAbstract::FEATURE
      ,BAD_INIT
      ,UNCOMPATIBLE_SIZE
    };

  static const std::string EXCEPTION_NAME;
  virtual const std::string& getExceptionName( void ) const { return ExceptionFeature::EXCEPTION_NAME; }

  ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
		     const std::string & msg = "" );

  ExceptionFeature ( const ExceptionFeature::ErrorCodeEnum& errcode,
			const std::string & msg,const char* format, ... );

  virtual ~ExceptionFeature( void ) throw() {}
};


} /* namespace sot */} /* namespace dynamicgraph */


#endif /* #ifndef __SOT_EXCEPTION_FEATURE_H */

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
