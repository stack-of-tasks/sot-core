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

#ifndef __SOT_MAILBOX_T_CPP
#define __SOT_MAILBOX_T_CPP

#include <sot/core/mailbox.hh>

namespace dynamicgraph {
  namespace sot {

    namespace dg = dynamicgraph;

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
      SOUT.setDependencyType( TimeDependency<int>::BOOL_DEPENDENT );
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
  
      if( lockMain.owns_lock() )
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
    get( typename Mailbox<Object>::sotTimestampedObject& res,const int& /*dummy*/ )
    {
      boost::timed_mutex::scoped_try_lock lockMain( this->mainObjectMutex );
  
      if( lockMain.owns_lock() )
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

    template< class Object >
    Object& Mailbox<Object>::
    getObject( Object& res,const int& time )
    { 
      const sotTimestampedObject & data = SOUT(time);
      res = data.obj; return res;
    }

    template< class Object >
    timeval& Mailbox<Object>::
    getTimestamp( struct timeval& res,const int& time )
    { 
      const sotTimestampedObject & data = SOUT(time);
      res.tv_sec = data.timestamp.tv_sec ; 
      res.tv_usec = data.timestamp.tv_usec ; 
      return res;
    }

  } /* namespace sot */
} /* namespace dynamicgraph */
/* Macro for template specialization */
#ifndef WIN32
# define MAILBOX_TEMPLATE_SPE(S)			\
  namespace dynamicgraph {                      \
    namespace sot {							\
      template void Mailbox<S>::post(const S& obj );			\
      template maal::boost::Vector&  Mailbox<S>::getObject( S& res,const int& time ); \
      template bool Mailbox<S>::hasBeenUpdated(void);			\
      template Mailbox<S>::~Mailbox();					\
      template Mailbox<S>::sotTimestampedObject& Mailbox<S>::get( Mailbox<S>::sotTimestampedObject& res,const int& dummy ); \
      template Mailbox<S>::Mailbox(const std::string& name);		\
  } \
} // namespace sot namespace dynamicgraph
#endif // WIN32


#endif // #ifdef __SOT_MAILBOX_T_CPP

