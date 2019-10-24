/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef UTILSWINDOWS_H_
#define UTILSWINDOWS_H_

#ifdef WIN32

#include "sot/core/api.hh"

#include < time.h >
#define NOMINMAX
#include <Winsock2.h>

struct SOT_CORE_EXPORT timezone {
  int tz_minuteswest; /* minutes W of Greenwich */
  int tz_dsttime;     /* type of dst correction */
};

int SOT_CORE_EXPORT gettimeofday(struct timeval *tv, struct timezone *tz);

#endif /*WIN32*/

#endif /* UTILSWINDOWS_H_ */
