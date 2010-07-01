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

#ifdef  HAVE_LIBBOOST_THREAD

/* --- SOT PLUGIN  --- */
#include <sot-core/mailbox.h>
#include <sot-core/debug.h>
#include <sot-core/factory.h>

using namespace sot;

typedef Mailbox<maal::boost::Vector> mailvect;
template<>SOT_FACTORY_ENTITY_PLUGIN(mailvect,"Mailbox<Vector>");


#endif // #ifdef  HAVE_LIBBOOST_THREAD


