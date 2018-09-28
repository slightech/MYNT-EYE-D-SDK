#ifndef HID_H_
#define HID_H_

#include <usb.h>
#include <memory>

#ifdef __cplusplus
extern "C" {
#endif

int rawhid_open(int max, int vid, int pid, int usage_page, int usage);
int rawhid_recv(int num, void *buf, int len, int timeout);
int rawhid_send(int num, void *buf, int len, int timeout);
void rawhid_close(int num);
int get_DeviceClass(int num);
void rawhid_droped();

#ifdef __cplusplus
}
#endif


namespace hid {

class hid_device {
public:
  using hid_t = struct hid_struct;

  using usb_device_t = struct usb_device;
  using usb_bus_t = struct usb_bus;
  using usb_interface_t = struct usb_interface;
  using usb_inter_desc_t = struct usb_interface_descriptor;
  using usb_end_desc_t = struct usb_endpoint_descriptor;

  struct hid_struct {
    usb_dev_handle *usb;
    int open;
    int iface;
    int ep_in;
    int ep_out;
    hid_t *prev;
    hid_t *next;
  };

  hid_device();
  virtual ~hid_device();

  int open(int max, int vid, int pid, int usage_page, int usage);
  int receive(int num, void* buf, int len, int timeout);
  int send(int num, void* buf, int len, int timeout);
  void close(int num);
  void droped();
  int get_device_class();

protected:
  void add_hid(hid_t *hid);
  hid_t *get_hid(int num);
  void free_all_hid(void);
  void hid_close(hid_t *hid);
  int hid_parse_item(uint32_t *val, uint8_t **data, const uint8_t *end);
  void process_usb_dev(usb_device_t *dev);

private:
  hid_t *first_hid_;
  hid_t *last_hid_;
  usb_device_t *first_dev_;

  bool is_opened_;
};

}  // namespace hid

#endif //HID_H_

