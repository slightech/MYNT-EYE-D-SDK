#ifndef STRING_HPP_
#define STRING_HPP_
#pragma once

#include <cstdio>
#include <memory>
#include <sstream>
#include <string>

// http://stackoverflow.com/questions/22774009/android-ndk-stdto-string-support
template<typename T>
std::string to_string(const T &value) {
    std::ostringstream os;
    os << value;
    return os.str();
}

template<typename T>
T Argument(T value) noexcept {
    return value;
}

template<typename T>
T const *Argument(std::basic_string<T> const &value) noexcept {
    return value.c_str();
}

inline const char *Argument(bool value) noexcept {
    return value ? "true" : "false";
}

// http://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
// Note: could not pass C++ string
template<typename ... Args>
std::string format_cstring(const std::string &format, const Args & ... args) {
    using namespace std;
    size_t size = snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
    unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args...);
    return string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

template<>
inline std::string format_cstring(const std::string &format) {
    return format;
}

// https://msdn.microsoft.com/en-us/magazine/dn913181.aspx
template<typename ... Args>
std::string format_string(const std::string &format, const Args & ... args) {
    return format_cstring(format, Argument(args)...);
}

template<>
inline std::string format_string(const std::string &format) {
    return format;
}

#endif  // STRING_HPP_
