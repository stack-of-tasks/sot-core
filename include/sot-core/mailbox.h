/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Mailbox.h
 * Project:   SOT
 * Author:    Nicolas Mansard
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

  Object& getObject( Object& res,const int& time )
    { 
      const sotTimestampedObject & data = SOUT(time);
      res = data.obj; return res;
    }
  struct timeval& getTimestamp( struct timeval& res,const int& time )
    { 
      const sotTimestampedObject & data = SOUT(time);
      res.tv_sec = data.timestamp.tv_sec ; 
      res.tv_usec = data.timestamp.tv_usec ; 
      return res;
    }

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


#include <sot-core/mailbox.t.cpp>

} // namespace sot

#endif // #ifdef  HAVE_LIBBOOST_THREAD
#endif // #ifndef  __SOT_MAILBOX_HH





