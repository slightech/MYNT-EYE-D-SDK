// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "mynteyed/data/hid/hid.h"
#include "mynteyed/util/log.h"

MYNTEYE_BEGIN_NAMESPACE

namespace hid {

hid_device::hid_device() :
  first_dev_(nullptr),
  first_hid_(nullptr),
  last_hid_(nullptr) {
}

hid_device::~hid_device() {
  free_all_hid();
}

int hid_device::get_device_class() {
  if (!first_dev_)
    return -1;

  return first_dev_->descriptor.bDeviceClass;
}

/**
 * receive - receive a packet
 *
 * Inputs:
 * num = device to receive from (zero based)
 * buf = buffer to receive packet
 * len = buffer's size
 * timeout = time to wait, in milliseconds
 *
 * Outputs:
 * number of bytes received, or -1 on error, -110 is timeout
 */
int hid_device::receive(int num, void *buf, int len, int timeout) {
  hid_t *hid = get_hid(num);

  if (!hid || !hid->open) {
    return -1;
  }
  return usb_bulk_read(hid->usb, 1, static_cast<char *>(buf), len, timeout);
}

/**
 * send - send a packet
 *
 * Inputs:
 * num = device to transmit to (zero based)
 * buf = buffer containing packet to send
 * len = number of bytes to transmit
 * timeout = time to wait, in milliseconds
 *
 * Outputs:
 * number of bytes sent, or -1 on error
 */
int hid_device::send(int num, void *buf, int len, int timeout) {
  hid_t *hid = get_hid(num);

  if (!hid || !hid->open) {
    return -1;
  }
  if (hid->ep_out) {
    return usb_bulk_write(hid->usb, hid->ep_out, static_cast<char *>(buf), len,
                          timeout);
  } else {
    return usb_control_msg(hid->usb, 0x21, 9, 0, hid->iface,
        static_cast<char *>(buf), len, timeout);
  }
}

/**
 * open - open 1 or more devices
 *
 * Inputs:
 * max = maximum number of devices to open
 * usage_page = top level usage page, or -1 if any
 * usage = top level usage number, or -1 if any
 *
 * Outputs:
 * actual number of devices opened
 */
int hid_device::open(int max, int usage_page, int usage) {
  if (first_hid_) {
    free_all_hid();
  }
  if (max < 1) {
    return 0;
  }

  // LOGI("hid_open, max = %d", max);
  usb_init();
  usb_find_busses();
  usb_find_devices();

  int count = -1;
  for (usb_bus_t *bus = usb_get_busses(); bus; bus = bus->next) {
    for (usb_device_t *dev = bus->devices; dev; dev = dev->next) {
      if (VID > 0 && dev->descriptor.idVendor != VID) {
        continue;
      }
      if (PID > 0 && dev->descriptor.idProduct != PID) {
        continue;
      }
      if (!dev->config || dev->config->bNumInterfaces < 1) {
        continue;
      }
      /*
      LOGI("device: vid = %04X, pic = %04X, with %d iface, bdeviceclass %d\n",
          dev->descriptor.idVendor, dev->descriptor.idProduct,
          dev->config->bNumInterfaces, dev->descriptor.bDeviceClass);
          */

      usb_interface_t *iface = dev->config->interface;
      usb_dev_handle *handle = nullptr;
      int claimed = 0;
      process_usb_dev(max, dev, iface, handle, count,
          claimed, usage, usage_page);
      if (handle && !claimed) {
        usb_close(handle);
      }
    }
  }
  return count;
}

/**
 * close - close a device
 *
 * Inputs:
 * num = device to close (zero based)
 *
 * Outputs:
 * nothings
 */
void hid_device::close(int num) {
  hid_t *hid = get_hid(num);

  if (!hid || !hid->open) {
    return;
  }
  hid_close(hid);
}

void hid_device::droped() {
  free_all_hid();
  first_dev_ = nullptr;
}

/**
 * Chuck Robey wrote a real HID report parser
 * (chuckr@telenix.org) chuckr@chuckr.org
 * http://people.freebsd.org/~chuckr/code/python/uhidParser-0.2.tbz
 * this tiny thing only needs to extract the top-level usage page
 * and usage, and even then is may not be truly correct, but it does
 * work with the Teensy Raw HID example.
 */
int hid_device::hid_parse_item(uint32_t *val, uint8_t **data,
                               const uint8_t *end) {
  const uint8_t *p = *data;
  uint8_t tag;
  int table[4] = {0, 1, 2, 4};
  int len;

  if (p >= end) return -1;
  if (p[0] == 0xFE) {
    // long item, HID 1.11, 6.2.2.3, page 27
    if (p + 5 >= end || p + p[1] >= end) return -1;
    tag = p[2];
    *val = 0;
    len = p[1] + 5;
  } else {
    // short item, HID 1.11, 6.2.2.2, page 26
    tag = p[0] & 0xFC;
    len = table[p[0] & 0x03];
    if (p + len + 1 >= end) return -1;
    switch (p[0] & 0x03) {
      case 3: *val = p[1] | (p[2] << 8) | (p[3] << 16) | (p[4] << 24); break;
      case 2: *val = p[1] | (p[2] << 8); break;
      case 1: *val = p[1]; break;
      case 0: *val = 0; break;
    }
  }
  *data += len + 1;
  return tag;
}

void hid_device::add_hid(hid_t *hid) {
  if (!first_hid_ || !last_hid_) {
    first_hid_ = last_hid_ = hid;
    hid->next = hid->prev = nullptr;
    return;
  }
  last_hid_->next = hid;
  hid->prev = last_hid_;
  hid->next = nullptr;
  last_hid_ = hid;
}

hid::hid_t *hid_device:: get_hid(int num) {
  hid_t *p;
  for (p = first_hid_; p && num > 0; p = p->next, num--) ;
  return p;
}

void hid_device::free_all_hid(void) {
  hid_t *p, *q;

  for (p = first_hid_; p; p = p->next) {
    hid_close(p);
  }
  p = first_hid_;
  while (p) {
    q = p;
    p = p->next;
    free(q);
  }
  first_hid_ = last_hid_ = nullptr;
}

void hid_device::hid_close(hid_t *hid) {
  if (hid->usb == nullptr)
    return;

  hid_t *p;
  int others = 0;

  usb_release_interface(hid->usb, hid->iface);
  for (p = first_hid_; p; p = p->next) {
    if (p->open && p->usb == hid->usb) others++;
  }
  if (!others) usb_close(hid->usb);
  hid->usb = nullptr;
  first_dev_ = nullptr;
}

void hid_device::process_usb_dev(int max,
                          usb_device_t *dev,
                          usb_interface_t *iface,
                          usb_dev_handle *handle,
                          int &count,
                          int &claimed,
                          int usage,
                          int usage_page) {
  for (int i = 0; i < dev->config->bNumInterfaces && iface; i++, iface++) {
    usb_inter_desc_t *desc = iface->altsetting;
    if (!desc) {
      continue;
    }
    // LOGI("  Type %d, %d, %d", desc->bInterfaceClass,
        // desc->bInterfaceSubClass, desc->bInterfaceProtocol);
    if (3 != desc->bInterfaceClass ||
        0 != desc->bInterfaceSubClass ||
        0 != desc->bInterfaceProtocol)
      continue;

    usb_end_desc_t *endp = desc->endpoint;
    int in = 0;
    int out = 0;
    for (int n = 0; n < desc->bNumEndpoints; n++, endp++) {
      // LOGI("%d \r", endp->bEndpointAddress);
      if (endp->bEndpointAddress & 0x80) {
        in = endp->bEndpointAddress & 0x7F;
        // LOGI("      IN endpoint %d", in);
      } else {
        out = endp->bEndpointAddress;
        // LOGI("      OUT endpoint %d\n", out);
      }
    }
    if (!in) {
      continue;
    }
    if (!handle) {
      handle = usb_open(dev);
      if (!handle) {
        // LOGI("    Unable to open device");
        break;
      }
    }
    // LOGI("   Hid interface (generic)");
    uint8_t buf[1024];
    if (usb_get_driver_np(handle, i,
          (char *)buf, sizeof(buf)) >= 0) {  // NOLINT
      // LOGI("  In use by driver \"%s\"", buf);
      if (usb_detach_kernel_driver_np(handle, i) < 0) {
        // LOGI("  Unable to detach from kernel\n");
      }
    }
    if (usb_claim_interface(handle, i) < 0) {
      // LOGI("  Unable claim interface %d", i);
      continue;
    }
    int len = usb_control_msg(handle, 0x81, 6, 0x2200, i,
      (char *)buf, sizeof(buf), 250); // NOLINT
    // LOGI("  descriptor, len=%d", len);
    if (len < 2) {
      usb_release_interface(handle, i);
      continue;
    }
    std::uint8_t *p = buf;
    int parsed_usage_page = 0;
    int parsed_usage = 0;
    std::uint32_t val = 0;
    int tag;
    while ((tag = hid_parse_item(&val, &p, buf + len)) >= 0) {
      if (tag == 4) {
        parsed_usage_page = val;
      }
      if (tag == 8) {
        parsed_usage = val;
      }
      if (parsed_usage && parsed_usage_page) {
        break;
      }
    }
    if ((!parsed_usage_page) || (!parsed_usage) ||
        (usage_page > 0 && parsed_usage_page != usage_page) ||
        (usage > 0 && parsed_usage != usage)) {
      usb_release_interface(handle, i);
      continue;
    }

    hid_t *hid = static_cast<hid_t *>(malloc(sizeof(hid_t)));
    if (!hid) {
      usb_release_interface(handle, i);
      continue;
    }
    hid->usb = handle;
    hid->iface = i;
    hid->ep_in = in;
    hid->ep_out = out;
    hid->open = 1;
    first_dev_ = dev;
    add_hid(hid);
    claimed++;
    count++;
    if (count >= max) {
      return;
    }
  }
}

bool hid_device::find_device() {
  usb_init();
  usb_find_busses();
  usb_find_devices();

  for (usb_bus_t *bus = usb_get_busses(); bus; bus = bus->next) {
    for (usb_device_t *dev = bus->devices; dev; dev = dev->next) {
      if (VID > 0 && dev->descriptor.idVendor != VID) {
        continue;
      }
      if (PID > 0 && dev->descriptor.idProduct != PID) {
        continue;
      }

      return true;
    }
  }

  return false;
}

int hid_device::get_version_number() {
  if (!first_hid_) { return -1; }

  return first_dev_->descriptor.bcdDevice;
}

}  // namespace hid

MYNTEYE_END_NAMESPACE
