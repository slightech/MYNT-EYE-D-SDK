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
#ifndef MYNTEYE_DATA_HID_HID_H_
#define MYNTEYE_DATA_HID_HID_H_
#pragma once

#include <memory>

#include "mynteyed/stubs/global.h"

#ifdef MYNTEYE_OS_WIN
#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>
#include <hidclass.h>

#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "hid.lib")
#endif

#ifdef MYNTEYE_OS_LINUX
#include <usb.h>
#endif

MYNTEYE_BEGIN_NAMESPACE

namespace hid {

const int VID = 0x0483;
const int PID = 0x5720;

typedef struct hid_struct {
#ifdef MYNTEYE_OS_WIN
  HANDLE handle;
  USHORT VersionNumber;
#else
  usb_dev_handle *usb;
  int ep_in;
  int ep_out;
  int iface;
#endif
  int open;
  struct hid_struct *prev;
  struct hid_struct *next;
} hid_t;

class hid_device {
 public:
#ifdef MYNTEYE_OS_LINUX
  using usb_device_t = struct usb_device;
  using usb_bus_t = struct usb_bus;
  using usb_interface_t = struct usb_interface;
  using usb_inter_desc_t = struct usb_interface_descriptor;
  using usb_end_desc_t = struct usb_endpoint_descriptor;
#endif

  hid_device();
  virtual ~hid_device();

  int open(int max, int usage_page, int usage);
  int receive(int num, void* buf, int len, int timeout);
  int send(int num, void* buf, int len, int timeout);
  void close(int num);
  void droped();
  int get_device_class();
  bool find_device();
  int get_version_number();

 protected:
  void add_hid(hid_t *hid);
  hid_t *get_hid(int num);
  void free_all_hid(void);
  void hid_close(hid_t *hid);
  int hid_parse_item(uint32_t *val, uint8_t **data, const uint8_t *end);
#ifdef MYNTEYE_OS_LINUX
  void process_usb_dev(int max,
      usb_device_t *dev,
      usb_interface_t *iface,
      usb_dev_handle *handle,
      int &count,    // NOLINT
      int &claimed,  // NOLINT
      int usage,
      int usage_page);
#endif

 private:
#ifdef MYNTEYE_OS_WIN
  HANDLE rx_event_;
  HANDLE tx_event_;
  CRITICAL_SECTION rx_mutex_;
  CRITICAL_SECTION tx_mutex_;
#else
  usb_device_t *first_dev_;
#endif
  hid_t *first_hid_;
  hid_t *last_hid_;
};

}  // namespace hid

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_DATA_HID_HID_H_
