// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SDR__VISIBILITY_CONTROL_H_
#define SDR__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SDR_EXPORT __attribute__ ((dllexport))
    #define SDR_IMPORT __attribute__ ((dllimport))
  #else
    #define SDR_EXPORT __declspec(dllexport)
    #define SDR_IMPORT __declspec(dllimport)
  #endif
  #ifdef SDR_BUILDING_DLL
    #define SDR_PUBLIC SDR_EXPORT
  #else
    #define SDR_PUBLIC SDR_IMPORT
  #endif
  #define SDR_PUBLIC_TYPE SDR_PUBLIC
  #define SDR_LOCAL
#else
  #define SDR_EXPORT __attribute__ ((visibility("default")))
  #define SDR_IMPORT
  #if __GNUC__ >= 4
    #define SDR_PUBLIC __attribute__ ((visibility("default")))
    #define SDR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SDR_PUBLIC
    #define SDR_LOCAL
  #endif
  #define SDR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // SDR__VISIBILITY_CONTROL_H_

