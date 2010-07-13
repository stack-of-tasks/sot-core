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




#ifndef __SOT_MAILBOX_VECTOR_HH
#define __SOT_MAILBOX_VECTOR_HH

#ifdef  HAVE_LIBBOOST_THREAD

/* --- SOT PLUGIN  --- */
#include <sot-core/mailbox.hxx>

#include <MatrixAbstractLayer/boost.h>

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

namespace sot {
	typedef Mailbox<maal::boost::Vector> MailboxVector;
}

#endif // #ifdef  HAVE_LIBBOOST_THREAD
#endif // #ifndef  __SOT_MAILBOX_HH





