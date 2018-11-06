#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mynteye_depth" for configuration "Release"
set_property(TARGET mynteye_depth APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mynteye_depth PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${MYNTEYED_SDK_ROOT}/lib/mynteye_depth.lib"
  IMPORTED_LOCATION_RELEASE "${MYNTEYED_SDK_ROOT}/bin/mynteye_depth.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS mynteye_depth )
list(APPEND _IMPORT_CHECK_FILES_FOR_mynteye_depth "${MYNTEYED_SDK_ROOT}/lib/mynteye_depth.lib" "${MYNTEYED_SDK_ROOT}/bin/mynteye_depth.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
