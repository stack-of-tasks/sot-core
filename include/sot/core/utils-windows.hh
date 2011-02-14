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

#ifndef UTILSWINDOWS_H_
#define UTILSWINDOWS_H_

#ifdef WIN32

#include "sot/core/api.hh"

#include < time.h >
#define NOMINMAX
#include <Winsock2.h>

struct SOT_CORE_EXPORT timezone
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};

int SOT_CORE_EXPORT gettimeofday(struct timeval *tv, struct timezone *tz);

#endif /*WIN32*/

#endif /* UTILSWINDOWS_H_ */
