/*
 * utils-windows.h
 *
 *  Created on: Jun 25, 2010
 *      Author: blue
 */

#ifndef UTILSWINDOWS_H_
#define UTILSWINDOWS_H_

#ifdef WIN32

#include <sot-core/sot-core-api.h>

#include < time.h >

struct SOT_CORE_EXPORT timezone
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};

int SOT_CORE_EXPORT gettimeofday(struct timeval *tv, struct timezone *tz);

#endif /*WIN32*/

#endif /* UTILSWINDOWS_H_ */
