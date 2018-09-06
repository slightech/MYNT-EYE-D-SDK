/**
 * <p>Macros:</p>
 * <ul>
 * <li>LOG_TAG: Log tag on Android
 * <li>LOG*: Log
 * <li>DBG_LOG*: Log when DEBUG
 * <li>DEBUG: Turn DBG_LOG* on
 * </ul>
 */
#ifndef LOG_HPP_
#define LOG_HPP_
#pragma once

#include "global.hpp"
#include "string.hpp"

#include <mutex>

namespace {

std::mutex &__get_mutex() {
    static std::mutex mtx;
    return mtx;
}

}  // namespace

#ifdef OS_ANDROID

#include <android/log.h>

#ifndef LOG_TAG
#define LOG_TAG "LOG_TAG"
#endif

inline int __log_print(const int &prio, const std::string &s) {
    std::lock_guard<std::mutex> lock(__get_mutex());
    return __android_log_write(prio, LOG_TAG, s.c_str());
}

template<typename T>
void log_print(const int &prio, const T &value) noexcept {
    __log_print(prio, to_string(value));
}

template<>
inline void log_print(const int &prio, const std::string &value) noexcept {
    __log_print(prio, to_string(value));
}

template<typename ... Args>
void log_print(const int &prio,
               const std::string &format,
               const Args & ... args) noexcept {
    __log_print(prio, format_string(format, args...));
}

#define LOGV(...) log_print(ANDROID_LOG_VERBOSE, __VA_ARGS__)
#define LOGD(...) log_print(ANDROID_LOG_DEBUG, __VA_ARGS__)
#define LOGI(...) log_print(ANDROID_LOG_INFO, __VA_ARGS__)
#define LOGW(...) log_print(ANDROID_LOG_WARN, __VA_ARGS__)
#define LOGE(...) log_print(ANDROID_LOG_ERROR, __VA_ARGS__)

#elif defined(OS_IPHONE)

#include <CoreFoundation/CoreFoundation.h>

extern "C" {
    void NSLog(CFStringRef format, ...);
    void CLSLog(CFStringRef format, ...);
}

inline void __log_print(const std::string &s) {
    std::lock_guard<std::mutex> lock(__get_mutex());
    CFStringRef str = CFStringCreateWithCString(
        kCFAllocatorDefault, s.c_str(), kCFStringEncodingUTF8);
    NSLog(str);
    CFRelease(str);
}

template<typename T>
void log_print(const T &value) noexcept {
    __log_print(to_string(value));
}

template<>
inline void log_print(const std::string &value) noexcept {
    __log_print(to_string(value));
}

template<typename ... Args>
void log_print(const std::string &format,
               const Args & ... args) noexcept {
    __log_print(format_string(format, args...));
}

#define LOGV(...) log_print(__VA_ARGS__)
#define LOGD(...) log_print(__VA_ARGS__)
#define LOGI(...) log_print(__VA_ARGS__)
#define LOGW(...) log_print(__VA_ARGS__)
#define LOGE(...) log_print(__VA_ARGS__)

#else // Non-mobile platform

#include <iostream>

template<typename T>
std::ostream &log_print(std::ostream &os,
                        const T &value) noexcept {
    std::lock_guard<std::mutex> lock(__get_mutex());
    os << value << std::endl;
    return os;
}

template<>
inline std::ostream &log_print(std::ostream &os,
                               const std::string &value) noexcept {
    std::lock_guard<std::mutex> lock(__get_mutex());
    os << value << std::endl;
    return os;
}

template<typename ... Args>
std::ostream &log_print(std::ostream &os,
                        const std::string &format,
                        const Args & ... args) noexcept {
    std::lock_guard<std::mutex> lock(__get_mutex());
    //os << sizeof...(args) << std::endl;
    os << format_string(format, args...) << std::endl;
    return os;
}

#define LOGV(...) log_print(std::clog, __VA_ARGS__)
#define LOGD(...) log_print(std::clog, __VA_ARGS__)
#define LOGI(...) log_print(std::clog, __VA_ARGS__)
#define LOGW(...) log_print(std::clog, __VA_ARGS__)
#define LOGE(...) log_print(std::cerr, __VA_ARGS__)

#endif

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

#endif  // GLOBAL_HPP_
