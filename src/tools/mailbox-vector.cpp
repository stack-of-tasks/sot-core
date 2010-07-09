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
#include <MatrixAbstractLayer/boost.h>
#include <sot-core/debug.h>
#include <sot-core/factory.h>
#include <sot-core/mailbox.t.cpp>
#include <sot-core/mailbox-vector.h>

using namespace sot;
using namespace dynamicgraph;
typedef Mailbox<maal::boost::Vector> mailvect;
template<>DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(mailvect,"Mailbox<Vector>");

MailboxVector::MailboxVector( const std::string& name): Mailbox(name){}

#endif // #ifdef  HAVE_LIBBOOST_THREAD


