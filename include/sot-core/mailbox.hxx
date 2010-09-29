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

#ifndef __SOT_MAILBOX_HH
#define __SOT_MAILBOX_HH

#ifdef  HAVE_LIBBOOST_THREAD


/* --- SOT PLUGIN  --- */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/all-signals.h>

/* --- BOOST --- */
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/xtime.hpp>

/* --- STD --- */
#include <time.h>
#ifndef WIN32
#include <sys/time.h>
#else
#include <sot-core/utils-windows.h>
#endif /*WIN32*/
#include <string>

namespace sot {

namespace dg = dynamicgraph;

template< class Object >
class Mailbox
: public dg::Entity
{
 public: 
  static const std::string CLASS_NAME;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public:
  struct sotTimestampedObject
  {
    Object obj;
    struct timeval timestamp;
  };

public:
  
  Mailbox( const std::string& name );
  ~Mailbox( void );
  
  void post( const Object& obj );
  sotTimestampedObject& get( sotTimestampedObject& res,const int& dummy );

  Object& getObject( Object& res,const int& time );
  struct timeval& getTimestamp( struct timeval& res,const int& time );

  bool hasBeenUpdated( void );
  
protected:

  boost::timed_mutex mainObjectMutex;
  Object mainObject;
  struct timeval mainTimeStamp;
  bool update;

 public: /* --- SIGNALS --- */

  dg::SignalTimeDependent< struct sotTimestampedObject,int > SOUT;
  dg::SignalTimeDependent< Object,int > objSOUT;
  dg::SignalTimeDependent< struct timeval,int > timeSOUT;

};



} // namespace sot

#endif // #ifdef  HAVE_LIBBOOST_THREAD
#endif // #ifndef  __SOT_MAILBOX_HH





