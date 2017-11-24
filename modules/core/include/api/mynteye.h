/**
 * @defgroup enumerations Public enumeration types
 */
#ifndef MYNTEYE_API_MYNTEYE_H_
#define MYNTEYE_API_MYNTEYE_H_
#pragma once

#include <cstdint>
#include <ostream>

#define STRINGIFY_HELPER(X) #X
#define STRINGIFY(X) STRINGIFY_HELPER(X)

#ifdef _WIN32
    #define DECL_EXPORT __declspec(dllexport)
    #define DECL_IMPORT __declspec(dllimport)
    #define DECL_HIDDEN
#else
    #define DECL_EXPORT __attribute__((visibility("default")))
    #define DECL_IMPORT __attribute__((visibility("default")))
    #define DECL_HIDDEN __attribute__((visibility("hidden")))
#endif

#ifdef DOXYGEN_WORKING

#define MYNTEYE_API

#else

#ifdef MYNTEYE_EXPORTS
    #define MYNTEYE_API DECL_EXPORT
#else
    #define MYNTEYE_API DECL_IMPORT
#endif

#endif

#if defined(CXX11) || defined(CXX0X)
    #define ENUM(name) enum class name : std::int8_t
#else
    #define ENUM(name) enum name
#endif

/**
 * MYNT EYE namespace
 */
namespace mynteye {

/**
 * @ingroup enumerations
 * @brief List error codes.
 */
ENUM(ErrorCode) {
    /** Standard code for successful behavior. */
    SUCCESS = 0,
    /** Standard code for unsuccessful behavior. */
    ERROR_FAILURE,
    /** File cannot be opened for not exist, not a regular file or any other reason. */
    ERROR_FILE_OPEN_FAILED,
    /** Camera cannot be opened for not plugged or any other reason. */
    ERROR_CAMERA_OPEN_FAILED,
    /** Camera is not opened now. */
    ERROR_CAMERA_NOT_OPENED,
    /** Camera retrieve the image failed. */
    ERROR_CAMERA_RETRIEVE_FAILED,
    /** Last guard. */
    ERROR_LAST
};

/**
 * @ingroup enumerations
 * @brief List available views.
 */
//ENUM(View) {
//    /** Image. */
//    VIEW_IMAGE,
//    /** Depth. */
//    VIEW_DEPTH,
//    /** Last guard. */
//    VIEW_LAST
//};

/**
 * @ingroup enumerations
 * @brief List device types.
 */
ENUM(DeviceType) {
    DEVICE_OTHERS = 0,
    DEVICE_AXES1,
    DEVICE_PUMA,
    DEVICE_KIWI,
    DEVICE_LAST
};

/**
 * @ingroup enumerations
 * @brief List depth modes.
 */
ENUM(DepthMode) {
    DEPTH_NON,
    DEPTH_GRAY,
    DEPTH_COLORFUL,
    DEPTH_NON_16UC1,
    DEPTH_NON_8UC1,
    DEPTH_LAST
};

/**
 * @ingroup enumerations
 * @brief List depth modes.
 */
ENUM(StreamFormat) {
    STREAM_MJPG,
    STREAM_YUYV,
    STREAM_LAST
};

}  // namespace mynteye

MYNTEYE_API std::ostream &operator<<(std::ostream &os, const mynteye::StreamFormat &code);

#endif  // MYNTEYE_API_MYNTEYE_H_
