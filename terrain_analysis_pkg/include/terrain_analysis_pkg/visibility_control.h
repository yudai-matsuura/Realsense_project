#ifndef TERRAIN_ANALYSIS_PKG__VISIBILITY_CONTROL_H_
#define TERRAIN_ANALYSIS_PKG__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TERRAIN_ANALYSIS_PKG_EXPORT __attribute__ ((dllexport))
    #define TERRAIN_ANALYSIS_PKG_IMPORT __attribute__ ((dllimport))
  #else
    #define TERRAIN_ANALYSIS_PKG_EXPORT __declspec(dllexport)
    #define TERRAIN_ANALYSIS_PKG_IMPORT __declspec(dllimport)
  #endif
  #ifdef TERRAIN_ANALYSIS_PKG_BUILDING_LIBRARY
    #define TERRAIN_ANALYSIS_PKG_PUBLIC TERRAIN_ANALYSIS_PKG_EXPORT
  #else
    #define TERRAIN_ANALYSIS_PKG_PUBLIC TERRAIN_ANALYSIS_PKG_IMPORT
  #endif
#else
  #define TERRAIN_ANALYSIS_PKG_EXPORT __attribute__ ((visibility ("default")))
  #define TERRAIN_ANALYSIS_PKG_IMPORT
  #if __GNUC__ >= 4
    #define TERRAIN_ANALYSIS_PKG_PUBLIC __attribute__ ((visibility ("default")))
    #define TERRAIN_ANALYSIS_PKG_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define TERRAIN_ANALYSIS_PKG_PUBLIC
    #define TERRAIN_ANALYSIS_PKG_LOCAL
  #endif
#endif

#endif  // TERRAIN_ANALYSIS_PKG__VISIBILITY_CONTROL_H_

