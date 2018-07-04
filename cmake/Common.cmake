include(${CMAKE_CURRENT_LIST_DIR}/Utils.cmake)

# OS specific instructions in CMAKE: How to?
#   https://stackoverflow.com/questions/9160335/os-specific-instructions-in-cmake-how-to
if(MSVC OR MSYS OR MINGW)
  set(OS_WIN TRUE)
  add_definitions(-DOS_WIN)
elseif(APPLE)
  set(OS_MAC TRUE)
elseif(UNIX)
  set(OS_LINUX TRUE)
else()
  message(STATUS "Maybe not work on this os.")
endif()

if(OS_LINUX)
  execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
  message(STATUS "ARCHITECTURE: ${ARCHITECTURE}")

  if(${ARCHITECTURE} STREQUAL "x86_64")
    set(ARCH_X64 TRUE)
  elseif(${ARCHITECTURE} STREQUAL "aarch64")
    set(ARCH_AARCH64 TRUE)
  elseif(${ARCHITECTURE} STREQUAL "arm")
    set(ARCH_ARM TRUE)
  elseif(${ARCHITECTURE} MATCHES "^(x86|i686|i386)$")
    set(ARCH_x86 TRUE)
  endif()
endif()

if(OS_LINUX)
  execute_process(COMMAND uname -r COMMAND tr -d '\n' OUTPUT_VARIABLE UNAME_R_TEGRA)
  #message(STATUS "UNAME_R_TEGRA: ${UNAME_R_TEGRA}")
  if(${UNAME_R_TEGRA} MATCHES ".*(tegra|jetsonbot).*")
    set(OS_TEGRA TRUE)
    message(STATUS "OS_TEGRA: ${OS_TEGRA}")
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCC)
  # GCC 5 changes, Dual ABI: https://gcc.gnu.org/gcc-5/changes.html#libstdcxx
  #add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
endif()

macro(exe2bat exe_dir exe_name dll_search_paths)
  if(OS_WIN)
    message(STATUS "Generating ${exe_name}.bat")
    set(__exe_name ${exe_name})
    file(TO_NATIVE_PATH ${MYNTEYE_SDK_ROOT} __mynteye_sdk_root)
    set(__dll_relative_search_paths "")
    foreach(path ${dll_search_paths})
      file(RELATIVE_PATH __relative_path "${exe_dir}" "${path}")
      file(TO_NATIVE_PATH ${__relative_path} __relative_path)
      list(APPEND __dll_relative_search_paths ${__relative_path})
    endforeach()
    #message(STATUS __dll_relative_search_paths: "${__dll_relative_search_paths}")
    set(__dll_search_paths "${__dll_relative_search_paths}")
    configure_file(
      "${PRO_DIR}/cmake/templates/exe.bat.in"
      "${exe_dir}/${__exe_name}.bat"
    )
  endif()
endmacro()

# make_shared_library(NAME
#                     [SRCS src1 src2 ...]
#                     [LINKLIBS lib1 lib2 ...]
#                     [OUTDIR outdir])
macro(make_shared_library NAME)
  message(STATUS "Generating shared library ${NAME}")

  parse_arguments(THIS
    "SRCS;LINKLIBS;OUTDIR"
    "MODULARIZED"
    ${ARGN}
  )
  #message(STATUS "NAME: ${NAME}")
  #message(STATUS "THIS_SRCS: ${THIS_SRCS}")
  #message(STATUS "THIS_LINKLIBS: ${THIS_LINKLIBS}")
  #message(STATUS "THIS_OUTDIR: ${THIS_OUTDIR}")

  add_library(${NAME} SHARED ${THIS_SRCS})
  if(THIS_LINKLIBS)
    target_link_libraries(${NAME} ${THIS_LINKLIBS})
  endif()

  if(MINGW)
    # MSVC and MinGW DLLs
    #   http://www.mingw.org/wiki/msvc_and_mingw_dlls
    # How to use libraries compiled with MingW in MSVC?
    #   https://stackoverflow.com/questions/2529770/how-to-use-libraries-compiled-with-mingw-in-msvc
    if(NOT THIS_OUTDIR)
      set(THIS_OUTDIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
    endif()
    file(MAKE_DIRECTORY ${THIS_OUTDIR})
    set_target_properties(${NAME} PROPERTIES
      LINK_FLAGS "-Wl,--output-def,${THIS_OUTDIR}/${CMAKE_SHARED_LIBRARY_PREFIX}${NAME}.def"
    )
  endif()
endmacro()

# make_executable(NAME
#                 [SRCS src1 src2 ...]
#                 [LINKLIBS lib1 lib2 ...]
#                 [OUTDIR outdir]
#                 [THREAD])
macro(make_executable NAME)
  message(STATUS "Generating executable ${NAME}")

  parse_arguments(THIS
    "SRCS;LINKLIBS;OUTDIR"
    "THREAD"
    ${ARGN}
  )
  #message(STATUS "NAME: ${NAME}")
  #message(STATUS "THIS_SRCS: ${THIS_SRCS}")
  #message(STATUS "THIS_LINKLIBS: ${THIS_LINKLIBS}")
  #message(STATUS "THIS_OUTDIR: ${THIS_OUTDIR}")

  add_executable(${NAME} ${THIS_SRCS})
  if(THIS_LINKLIBS)
    target_link_libraries(${NAME} ${THIS_LINKLIBS})
  endif()
  if(THIS_OUTDIR)
    set_target_properties(${NAME} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY          "${THIS_OUTDIR}"
      RUNTIME_OUTPUT_DIRECTORY_DEBUG    "${THIS_OUTDIR}"
      RUNTIME_OUTPUT_DIRECTORY_RELEASE  "${THIS_OUTDIR}"
    )
    exe2bat("${THIS_OUTDIR}" "${NAME}" "${DLL_SEARCH_PATHS}")
  else()
    exe2bat("${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" "${NAME}" "${DLL_SEARCH_PATHS}")
  endif()
  if(THIS_THREAD)
    # cmake and libpthread
    #   https://stackoverflow.com/questions/1620918/cmake-and-libpthread
    find_package(Threads REQUIRED)
    if(THREADS_HAVE_PTHREAD_ARG)
      target_compile_options(PUBLIC ${NAME} "-pthread")
    endif()
    if(CMAKE_THREAD_LIBS_INIT)
      target_link_libraries(${NAME} "${CMAKE_THREAD_LIBS_INIT}")
    endif()
  endif()
endmacro()

# set_outdir(ARCHIVE_OUTPUT_DIRECTORY
#            LIBRARY_OUTPUT_DIRECTORY
#            RUNTIME_OUTPUT_DIRECTORY)
macro(set_outdir ARCHIVE_OUTPUT_DIRECTORY LIBRARY_OUTPUT_DIRECTORY RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
  # In CMake, how do I work around the Debug and Release directories Visual Studio 2010 tries to add?
  #   https://stackoverflow.com/questions/7747857/in-cmake-how-do-i-work-around-the-debug-and-release-directories-visual-studio-2
  foreach(CONFIG ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${CONFIG} CONFIG)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  endforeach()
  # For exe2bat
  if(OS_WIN)
    if(NOT PRO_DIR)
      message(FATAL_ERROR "PRO_DIR variable not found. Please set it.")
    endif()
    set(MYNTEYE_SDK_ROOT ${PRO_DIR}/output)
    set(WIN_DIR ${PRO_DIR}/platforms/win)
    if(MINGW)
      set(DLL_SEARCH_PATHS "${PRO_DIR}/output/bin;${PRO_DIR}/output/bin/3rdparty;${WIN_DIR}/deps/opencv/x64/mingw/bin")
    else()
      set(DLL_SEARCH_PATHS "${PRO_DIR}/output/bin;${PRO_DIR}/output/bin/3rdparty;${WIN_DIR}/deps/opencv/x64/vc14/bin")
    endif()
  endif()
endmacro()
