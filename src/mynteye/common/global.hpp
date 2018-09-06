#ifndef GLOBAL_HPP_
#define GLOBAL_HPP_
#pragma once

// http://stackoverflow.com/questions/5919996/
// how-to-detect-reliably-mac-os-x-ios-linux-windows-in-c-preprocessor
#ifdef _WIN32
    #define OS_WIN
    #ifdef _WIN64
        #define OS_WIN64
    #else
        #define OS_WIN32
    #endif
    #if defined(__MINGW32__) || defined(__MINGW64__)
        #define OS_MINGW
        #ifdef __MINGW64__
            #define OS_MINGW64
        #else
            #define OS_MINGW32
        #endif
    #endif
#elif __APPLE__
    #include "TargetConditionals.h"
    #if TARGET_IPHONE_SIMULATOR
        #define OS_IPHONE
        #define OS_IPHONE_SIMULATOR
    #elif TARGET_OS_IPHONE
        #define OS_IPHONE
    #elif TARGET_OS_MAC
        #define OS_MAC
    #else
        #error "Unknown Apple platform"
    #endif
#elif __ANDROID__
    #define OS_ANDROID
#elif __linux__
    #define OS_LINUX
#elif __unix__
    #define OS_UNIX
#elif defined(_POSIX_VERSION)
    #define OS_POSIX
#else
    #error "Unknown compiler"
#endif

// https://stackoverflow.com/questions/22285240/mingw-use-declspecdllexport-or-attribute-visibilitydefault
#ifdef OS_WIN
    #define DECL_EXPORT __declspec(dllexport)
    #define DECL_IMPORT __declspec(dllimport)
    #define DECL_HIDDEN
#else
    #define DECL_EXPORT __attribute__((visibility("default")))
    #define DECL_IMPORT __attribute__((visibility("default")))
    #define DECL_HIDDEN __attribute__((visibility("hidden")))
#endif

#if defined(OS_WIN) && !defined(OS_MINGW)
    #define OS_SEP "\\"
#else
    #define OS_SEP "/"
#endif

#define DISABLE_COPY(Class) \
    Class(const Class &) = delete; \
    Class &operator=(const Class &) = delete;

// http://stackoverflow.com/questions/15763937/unused-parameter-in-c11
template<typename ... T> void unused(T && ...) {}

#endif  // GLOBAL_HPP_
