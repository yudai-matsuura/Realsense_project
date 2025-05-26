// Copyright (c) 2024 Tohoku Univ. Space Robotics Lab.
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

#ifndef LBR_ARITHMETIC_AVERAGE_ROUGHNESS__VISIBILITY_CONTROL_H_
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_EXPORT __attribute__((dllexport))
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_IMPORT __attribute__((dllimport))
#else
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_EXPORT __declspec(dllexport)
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_IMPORT __declspec(dllimport)
#endif
#ifdef LBR_ARITHMETIC_AVERAGE_ROUGHNESS_BUILDING_LIBRARY
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_PUBLIC LBR_ARITHMETIC_AVERAGE_ROUGHNESS_EXPORT
#else
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_PUBLIC LBR_ARITHMETIC_AVERAGE_ROUGHNESS_IMPORT
#endif
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_PUBLIC_TYPE LBR_ARITHMETIC_AVERAGE_ROUGHNESS_PUBLIC
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_LOCAL
#else
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_EXPORT __attribute__((visibility("default")))
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_IMPORT
#if __GNUC__ >= 4
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_PUBLIC __attribute__((visibility("default")))
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_LOCAL __attribute__((visibility("hidden")))
#else
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_PUBLIC
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_LOCAL
#endif
#define LBR_ARITHMETIC_AVERAGE_ROUGHNESS_PUBLIC_TYPE
#endif

#endif  // LBR_ARITHMETIC_AVERAGE_ROUGHNESS__VISIBILITY_CONTROL_H_
