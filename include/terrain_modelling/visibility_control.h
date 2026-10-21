#ifndef TERRAIN_MODELLING__VISIBILITY_CONTROL_H_
#define TERRAIN_MODELLING__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TERRAIN_MODELLING_EXPORT __attribute__ ((dllexport))
    #define TERRAIN_MODELLING_IMPORT __attribute__ ((dllimport))
  #else
    #define TERRAIN_MODELLING_EXPORT __declspec(dllexport)
    #define TERRAIN_MODELLING_IMPORT __declspec(dllimport)
  #endif
  #ifdef TERRAIN_MODELLING_BUILDING_LIBRARY
    #define TERRAIN_MODELLING_PUBLIC TERRAIN_MODELLING_EXPORT
  #else
    #define TERRAIN_MODELLING_PUBLIC TERRAIN_MODELLING_IMPORT
  #endif
  #define TERRAIN_MODELLING_PUBLIC_TYPE TERRAIN_MODELLING_PUBLIC
  #define TERRAIN_MODELLING_LOCAL
#else
  #define TERRAIN_MODELLING_EXPORT __attribute__ ((visibility("default")))
  #define TERRAIN_MODELLING_IMPORT
  #if __GNUC__ >= 4
    #define TERRAIN_MODELLING_PUBLIC __attribute__ ((visibility("default")))
    #define TERRAIN_MODELLING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TERRAIN_MODELLING_PUBLIC
    #define TERRAIN_MODELLING_LOCAL
  #endif
  #define TERRAIN_MODELLING_PUBLIC_TYPE
#endif

#endif  // TERRAIN_MODELLING__VISIBILITY_CONTROL_H_
