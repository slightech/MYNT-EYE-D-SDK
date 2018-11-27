// Copyright 2018 Slightech Co., Ltd. All rights reserved.
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
/**
 * <p>Macros:</p>
 * <ul>
 * <li>LOG_TAG: Log tag on Android
 * <li>LOG*: Log
 * <li>DBG_LOG*: Log when DEBUG
 * <li>DEBUG: Turn DBG_LOG* on
 * </ul>
 */
#ifndef MYNTEYE_UTIL_LOG_H_
#define MYNTEYE_UTIL_LOG_H_
#pragma once

#include <iostream>
#include <mutex>
#include <string>

#include "mynteyed/stubs/global.h"
#include "mynteyed/util/strings.h"

MYNTEYE_BEGIN_NAMESPACE

inline std::mutex& __get_mutex() {
  static std::mutex mtx;
  return mtx;
}

#ifdef MYNTEYE_OS_ANDROID

#include <android/log.h>

#ifndef LOG_TAG
#define LOG_TAG "LOG_TAG"
#endif

inline int __log_print(const int& prio, const std::string& s) {
  std::lock_guard<std::mutex> lock(__get_mutex());
  return __android_log_write(prio, LOG_TAG, s.c_str());
}

template<typename T>
void log_print(const int& prio, const T& value) noexcept {
  __log_print(prio, strings::to_string(value));
}

template<>
inline void log_print(const int& prio, const std::string& value) noexcept {
  __log_print(prio, strings::to_string(value));
}

template<typename... Args>
void log_print(const int& prio, const std::string& format,
    const Args&... args) noexcept {
  __log_print(prio, strings::format_string(format, args...));
}

#define LOGV(...) log_print(ANDROID_LOG_VERBOSE, __VA_ARGS__)
#define LOGD(...) log_print(ANDROID_LOG_DEBUG, __VA_ARGS__)
#define LOGI(...) log_print(ANDROID_LOG_INFO, __VA_ARGS__)
#define LOGW(...) log_print(ANDROID_LOG_WARN, __VA_ARGS__)
#define LOGE(...) log_print(ANDROID_LOG_ERROR, __VA_ARGS__)

#elif defined(MYNTEYE_OS_IPHONE)

#include <CoreFoundation/CoreFoundation.h>

extern "C" {
  void NSLog(CFStringRef format, ...);
  void CLSLog(CFStringRef format, ...);
}

inline void __log_print(const std::string& s) {
  std::lock_guard<std::mutex> lock(__get_mutex());
  CFStringRef str = CFStringCreateWithCString(
    kCFAllocatorDefault, s.c_str(), kCFStringEncodingUTF8);
  NSLog(str);
  CFRelease(str);
}

template<typename T>
void log_print(const T& value) noexcept {
  __log_print(strings::to_string(value));
}

template<>
inline void log_print(const std::string& value) noexcept {
  __log_print(strings::to_string(value));
}

template<typename... Args>
void log_print(const std::string& format, const Args&... args) noexcept {
  __log_print(strings::format_string(format, args...));
}

#define LOGV(...) log_print(__VA_ARGS__)
#define LOGD(...) log_print(__VA_ARGS__)
#define LOGI(...) log_print(__VA_ARGS__)
#define LOGW(...) log_print(__VA_ARGS__)
#define LOGE(...) log_print(__VA_ARGS__)

#else  // Non-mobile platform

template<typename T>
std::ostream& log_print(std::ostream& os, const T& value) noexcept {
  std::lock_guard<std::mutex> lock(__get_mutex());
  os << value << std::endl;
  return os;
}

template<>
inline std::ostream& log_print(std::ostream& os,
    const std::string& value) noexcept {
  std::lock_guard<std::mutex> lock(__get_mutex());
  os << value << std::endl;
  return os;
}

template<typename... Args>
std::ostream& log_print(std::ostream& os, const std::string& format,
    const Args&... args) noexcept {
  std::lock_guard<std::mutex> lock(__get_mutex());
  // os << sizeof...(args) << std::endl;
  os << strings::format_string(format, args...) << std::endl;
  return os;
}

#define LOGV(...) log_print(std::clog, __VA_ARGS__)
#define LOGD(...) log_print(std::clog, __VA_ARGS__)
#define LOGI(...) log_print(std::clog, __VA_ARGS__)
#define LOGW(...) log_print(std::clog, __VA_ARGS__)
#define LOGE(...) log_print(std::cerr, __VA_ARGS__)

#endif

MYNTEYE_END_NAMESPACE

// Expose debug log values in debug
#ifdef DEBUG
#define DBG_LOGV(...) LOGV(__VA_ARGS__)
#define DBG_LOGD(...) LOGD(__VA_ARGS__)
#define DBG_LOGI(...) LOGI(__VA_ARGS__)
#define DBG_LOGW(...) LOGW(__VA_ARGS__)
#define DBG_LOGE(...) LOGE(__VA_ARGS__)
#else
#define DBG_LOGV(...)
#define DBG_LOGD(...)
#define DBG_LOGI(...)
#define DBG_LOGW(...)
#define DBG_LOGE(...)
#endif

#ifndef throw_error
#include <stdexcept>
#define throw_error(MSG) { LOGE(MSG); throw new std::runtime_error(MSG); }  // NOLINT
#endif

#endif  // MYNTEYE_UTIL_LOG_H_
