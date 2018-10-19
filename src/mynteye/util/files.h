#ifndef MYNTEYE_FILES_H // NOLINT
#define MYNTEYE_FILES_H

#include "mynteye/stubs/global.h"
#include "mynteye/util/strings.h"

#if defined(OS_WIN) && !defined(OS_MINGW) && !defined(OS_CYGWIN)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <string>

MYNTEYE_BEGIN_NAMESPACE

namespace files {

bool _mkdir(const std::string &path) {
#if defined(OS_MINGW) || defined(OS_CYGWIN)
  const int status = ::mkdir(path.c_str());
#elif defined(OS_WIN)
  const int status = ::_mkdir(path.c_str());
#else
  const int status =
    ::mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

  if (status != 0 && errno != EEXIST) {
    std::cout << "Create directory failed (status " << status
      << "), path: " << path << std::endl;
    return false;
  }
  if (errno == EEXIST) {
    std::cout << "Create directory needless (already exist), path: " << std::endl;
    return true;
  } else {
    std::cout << "Create directory success, path: " << std::endl;
    return true;
  }
}

bool mkdir(const std::string &path) {
  auto &&dirs = sstrings::split(path, MYNTEYE_OS_SEP);
  auto &&size = dirs.size();
  if (size <= 0)
    return false;
  std::string p{dirs[0]};
  if (!_mkdir(p))
    return false;
  for (std::size_t i = 1; i < size; i++) {
    p.append(MYNTEYE_OS_SEP).append(dirs[i]);
    if (!_mkdir(p))
      return false;
  }
  return true;
}

} // namespace files

MYNTEYE_END_NAMESPACE

#endif //MYNTEYE_FILES_H // NOLINT
