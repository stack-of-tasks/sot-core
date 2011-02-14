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

#ifndef SOT_CORE_API_HH
#define SOT_CORE_API_HH

#if defined (WIN32)
#  ifdef sot_core_EXPORTS
#    define SOT_CORE_EXPORT __declspec(dllexport)
#  else
#    define SOT_CORE_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOT_CORE_EXPORT
#endif

#endif
