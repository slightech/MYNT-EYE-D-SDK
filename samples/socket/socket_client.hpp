#ifndef SOCKET_CLIENT_HPP
#define SOCKET_CLIENT_HPP

#include <opencv2/core/core.hpp>

class SocketClient {
 public:
  SocketClient(const char* hostname, int port);
  void ConnectToServer();
  void SendImageDims(const int image_rows, const int image_cols);
  void SendImage(cv::Mat& image);
 private:
  const char* hostname_;
  int port_;
  int pic_num_;
  int socket_fdesc_;
};

#endif
