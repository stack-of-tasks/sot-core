/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      Mailbox.cpp
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

#include <sot/sotConfig.h>
#ifdef  HAVE_LIBBOOST_THREAD

/* --- SOT PLUGIN  --- */
#include <sot/Mailbox.h>

#include <sot/sotDebug.h>

#include <sot/sotFactory.h>


typedef Mailbox<maal::boost::Vector> mailvect;
template<>SOT_FACTORY_ENTITY_PLUGIN(mailvect,"Mailbox<Vector>");


#endif // #ifdef  HAVE_LIBBOOST_THREAD


