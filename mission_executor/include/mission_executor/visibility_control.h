#ifndef MISSION_EXECUTOR__VISIBILITY_CONTROL_H_
#define MISSION_EXECUTOR__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MISSION_EXECUTOR_EXPORT __attribute__ ((dllexport))
    #define MISSION_EXECUTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define MISSION_EXECUTOR_EXPORT __declspec(dllexport)
    #define MISSION_EXECUTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef MISSION_EXECUTOR_BUILDING_DLL
    #define MISSION_EXECUTOR_PUBLIC MISSION_EXECUTOR_EXPORT
  #else
    #define MISSION_EXECUTOR_PUBLIC MISSION_EXECUTOR_IMPORT
  #endif
  #define MISSION_EXECUTOR_PUBLIC_TYPE MISSION_EXECUTOR_PUBLIC
  #define MISSION_EXECUTOR_LOCAL
#else
  #define MISSION_EXECUTOR_EXPORT __attribute__ ((visibility("default")))
  #define MISSION_EXECUTOR_IMPORT
  #if __GNUC__ >= 4
    #define MISSION_EXECUTOR_PUBLIC __attribute__ ((visibility("default")))
    #define MISSION_EXECUTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MISSION_EXECUTOR_PUBLIC
    #define MISSION_EXECUTOR_LOCAL
  #endif
  #define MISSION_EXECUTOR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MISSION_EXECUTOR__VISIBILITY_CONTROL_H_