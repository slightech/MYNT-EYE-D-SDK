#include "socket_client.hpp"

#include <memory> // unique_ptr
#include <cstdio>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/types.h> 
#include <sys/socket.h>

#include <opencv2/highgui/highgui.hpp>

SocketClient::SocketClient(const char* hostname, int port) :
    hostname_ (hostname),
    port_(port),
    pic_num_(0),
    socket_fdesc_(0) {}

void SocketClient::ConnectToServer() {
  struct addrinfo addrinfo_hints;
  struct addrinfo* addrinfo_resp;

  // Specify criteria for address structs to be returned by getAddrinfo
  memset(&addrinfo_hints, 0, sizeof(addrinfo_hints));
  addrinfo_hints.ai_socktype = SOCK_STREAM;
  addrinfo_hints.ai_family = AF_INET;

  // Populate addr_info_resp with address responses matching hints
  if (getaddrinfo(hostname_, std::to_string(port_).c_str(),
                  &addrinfo_hints, &addrinfo_resp) != 0) {
    perror("Couldn't connect to host!");
    exit(1);
  }

  // Create socket file descriptor for server
  socket_fdesc_ = socket(addrinfo_resp->ai_family, addrinfo_resp->ai_socktype,
                        addrinfo_resp->ai_protocol);
  if (socket_fdesc_ == -1) {
    perror("Error opening socket");
    exit(1);
  }

  // Connect to server specified in address struct, assign process to server
  // file descriptor
  if (connect(socket_fdesc_, addrinfo_resp->ai_addr,
              addrinfo_resp->ai_addrlen) == -1) {
    perror("Error connecting to address");
    exit(1);
  }

  free(addrinfo_resp);
}

void SocketClient::SendImageDims(int cols, int rows) {
  // Send number of rows to server
  if (send(socket_fdesc_, (char*)&cols, sizeof(cols), 0) == -1) {
    perror("Error sending rows");
    exit(1);
  }

  // Send number of cols to server
  if (send(socket_fdesc_, (char*)&rows, sizeof(rows), 0) == -1) {
    perror("Error sending cols");
    exit(1);
  }
}

void SocketClient::SendImage(cv::Mat& image) {
  image = image.reshape(0,1);
  int image_size = image.total() * image.elemSize();
  int num_bytes = send(socket_fdesc_, image.data, image_size, 0);
  printf("Sent %d bytes of %d-byte image to port %d\n",
         num_bytes, image_size, port_);
}
