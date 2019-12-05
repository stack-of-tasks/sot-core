/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 */

#ifndef SOT_CORE_API_HH
#define SOT_CORE_API_HH

#if defined(WIN32)
#ifdef sot_core_EXPORTS
#define SOT_CORE_EXPORT __declspec(dllexport)
#else
#define SOT_CORE_EXPORT __declspec(dllimport)
#endif
#else
#define SOT_CORE_EXPORT
#endif

#endif
