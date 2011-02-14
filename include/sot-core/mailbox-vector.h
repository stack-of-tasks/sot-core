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

#ifndef __SOT_MAILBOX_VECTOR_HH
#define __SOT_MAILBOX_VECTOR_HH

/* --- SOT PLUGIN  --- */
#include <sot-core/mailbox.hxx>

#include <jrl/mal/boost.hh>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32) 
#  if defined (mailbox_vector_EXPORTS)
#    define MAILBOX_VECTOR_EXPORT __declspec(dllexport)
#  else  
#    define MAILBOX_VECTOR_EXPORT  __declspec(dllimport)
#  endif 
#else
#  define MAILBOX_VECTOR_EXPORT 
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

namespace dynamicgraph {
  namespace sot {
#ifdef WIN32
    class MAILBOX_VECTOR_EXPORT MailboxVector : public Mailbox<maal::boost::Vector> 
    {
    public:
      MailboxVector( const std::string& name );
    };
#else
    typedef Mailbox<maal::boost::Vector> MailboxVector;
#endif
  } // namespace sot
} // namespace dynamicgraph

#endif // #ifndef  __SOT_MAILBOX_HH





