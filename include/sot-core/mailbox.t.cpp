/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Mailbox.t.cpp
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




#ifndef __SOT_MAILBOX_T_CPP
#define __SOT_MAILBOX_T_CPP



/* --- SOT PLUGIN  --- */
#include <sot-core/mailbox.h>

//#undef VP_TEMPLATE_DEBUG
//#define VP_TEMPLATE_DEBUG_MODE             0
#include <sot-core/debug.h>

  
/* -------------------------------------------------------------------------- */
/* --- CONSTRUCTION --------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
template< class Object >
Mailbox<Object>::
Mailbox( const std::string& name )
  :Entity(name)
  ,mainObjectMutex()
  ,mainObject()
  ,update(false)

   ,SOUT( boost::bind(&Mailbox::get,this,_1,_2),
	  sotNOSIGNAL,
	  "Mailbox("+name+")::output(Object)::out" )
   ,objSOUT( boost::bind(&Mailbox::getObject,this,_1,_2),
	     SOUT,
	     "Mailbox("+name+")::output(Object)::object" )
   ,timeSOUT( boost::bind(&Mailbox::getTimestamp,this,_1,_2),
	      SOUT,
	      "Mailbox("+name+")::output(Object)::timestamp" )
{
  signalRegistration( SOUT<<objSOUT<<timeSOUT );
  SOUT.setDependancyType( TimeDependancy<int>::BOOL_DEPENDANT );
}

template< class Object >
Mailbox<Object>::
~Mailbox( void )
{
  boost::timed_mutex::scoped_lock lockMain( mainObjectMutex );
}

/* -------------------------------------------------------------------------- */
/* --- ACCESS --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
template< class Object >
bool Mailbox<Object>::
hasBeenUpdated( void )
{
  boost::timed_mutex::scoped_try_lock lockMain( this->mainObjectMutex );
  
  if( lockMain.locked() )
    {
      return update;
    }
  else 
    {
      return false;
    }
}


/* -------------------------------------------------------------------------- */
template< class Object >
typename Mailbox<Object>::sotTimestampedObject& Mailbox<Object>::
get( typename Mailbox<Object>::sotTimestampedObject& res,const int& dummy )
{
  boost::timed_mutex::scoped_try_lock lockMain( this->mainObjectMutex );
  
  if( lockMain.locked() )
    {
      res.timestamp.tv_sec=this->mainTimeStamp.tv_sec;
      res.timestamp.tv_usec=this->mainTimeStamp.tv_usec;
      
      update = false;
      res.obj = this->mainObject;
    }

  return res;
}

/* -------------------------------------------------------------------------- */
template< class Object >
void Mailbox<Object>::
post( const Object& value )
{
  boost::timed_mutex::scoped_lock lockMain( this->mainObjectMutex );
  mainObject = value;
  gettimeofday( &this->mainTimeStamp, NULL );
  update = true;
  SOUT.setReady();

  return;
}


#endif // #ifdef __SOT_MAILBOX_T_CPP

