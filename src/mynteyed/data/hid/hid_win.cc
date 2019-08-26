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

namespace {

void print_error() {
  char buf[256];

  DWORD error_no = GetLastError();
  FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM,
      nullptr, error_no, 0, buf, sizeof(buf), nullptr);
  // LOGI("error_no %ld: %s\n", error_no, buf);
}

}  // namespace

namespace hid {

hid_device::hid_device() : rx_event_(nullptr),
  tx_event_(nullptr),
  first_hid_(nullptr),
  last_hid_(nullptr) {
}

hid_device::~hid_device() {
  free_all_hid();
}

int hid_device::get_device_class() {
  unsigned char desc[253] = {'\0'};

  if (!first_hid_)
    return -1;

  if (HidD_GetProductString(first_hid_->handle, desc, sizeof(desc))) {
    if ('B' == desc[28]) { return 0xFF; }
  } else {
    return -1;
  }
  return 0;
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
  hid_t *hid;
  unsigned char read_buf[516];
  OVERLAPPED ov;
  DWORD n_size;

  if (sizeof(read_buf) < len + 1) { return -1; }
  hid = get_hid(num);
  if (!hid || !hid->open) { return -1; }
  EnterCriticalSection(&rx_mutex_);
  ResetEvent(&rx_event_);
  memset(&ov, 0, sizeof(ov));
  ov.hEvent = rx_event_;

  if (!ReadFile(hid->handle, read_buf, len + 1, NULL, &ov)) {
    if (GetLastError() != ERROR_IO_PENDING) { goto return_error; }
    DWORD ret = WaitForSingleObject(rx_event_, timeout);
    if (ret == WAIT_TIMEOUT) { goto return_timeout; }
    if (ret != WAIT_OBJECT_0) { goto return_error; }
  }

  if (!GetOverlappedResult(hid->handle,
        &ov, &n_size, false)) { goto return_error; }
  LeaveCriticalSection(&rx_mutex_);

  if (n_size <= 0) { return -1; }
  n_size--;
  if (n_size > len) { n_size = len; }
  memcpy(buf, read_buf + 1, n_size);
  return n_size;

return_timeout:
  CancelIo(hid->handle);
  LeaveCriticalSection(&rx_mutex_);
  return 0;
return_error:
  print_error();
  LeaveCriticalSection(&rx_mutex_);
  return -1;
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
  hid_t *hid;
  unsigned char write_buf[516];
  OVERLAPPED ov;
  DWORD n_size;

  if (sizeof(write_buf) < len + 1) { return -1; }
  hid = get_hid(num);
  EnterCriticalSection(&tx_mutex_);
  ResetEvent(&tx_event_);
  memset(&ov, 0, sizeof(ov));
  ov.hEvent = tx_event_;

  write_buf[0] = 0;
  memcpy(write_buf + 1, buf, len);
  if (!WriteFile(hid->handle, write_buf, len + 1, NULL, &ov)) {
    if (GetLastError() != ERROR_IO_PENDING) { goto return_error; }
    DWORD ret = WaitForSingleObject(tx_event_, timeout);
    if (ret == WAIT_TIMEOUT) { goto return_timeout; }
    if (ret != WAIT_OBJECT_0) { goto return_error; }
  }
  if (!GetOverlappedResult(hid->handle,
        &ov, &n_size, false)) { goto return_error; }
  LeaveCriticalSection(&tx_mutex_);
  if (n_size <= 0) { return -1; }
  return n_size - 1;

return_timeout:
  CancelIo(hid->handle);
  LeaveCriticalSection(&tx_mutex_);
  return 0;
return_error:
  print_error();
  LeaveCriticalSection(&tx_mutex_);
  return -1;
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
  if (first_hid_) { free_all_hid(); }
  if (max < 1) { return 0; }
  if (!rx_event_) {
    rx_event_ = CreateEvent(nullptr, true, true, nullptr);
    tx_event_ = CreateEvent(nullptr, true, true, nullptr);
    InitializeCriticalSection(&rx_mutex_);
    InitializeCriticalSection(&tx_mutex_);
  }

  GUID id;
  HidD_GetHidGuid(&id);
  HDEVINFO info = SetupDiGetClassDevs(&id,
      nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
  if (info == INVALID_HANDLE_VALUE) { return 0; }

  SP_DEVICE_INTERFACE_DATA iface;
  int count = 0;
  for (DWORD index = 0; ; index++) {
    iface.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
    BOOL ret = SetupDiEnumDeviceInterfaces(info,
        nullptr, &id, index, &iface);
    if (!ret) { return count; }

    DWORD reqd_size;
    SetupDiGetInterfaceDeviceDetail(info,
        &iface, nullptr, 0, &reqd_size, nullptr);

    SP_DEVICE_INTERFACE_DETAIL_DATA *details =
      (SP_DEVICE_INTERFACE_DETAIL_DATA *)malloc(reqd_size);   // NOLINT
    if (details == nullptr) { continue; }

    memset(details, 0, reqd_size);
    details->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
    ret = SetupDiGetDeviceInterfaceDetail(info,
        &iface, details, reqd_size, nullptr, nullptr);
    if (!ret) {
      free(details);
      continue;
    }

    HANDLE handle = CreateFile(details->DevicePath,
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr, OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED, nullptr);
    free(details);
    if (handle == INVALID_HANDLE_VALUE) { continue; }

    HIDD_ATTRIBUTES attrib;
    PHIDP_PREPARSED_DATA hid_data;
    attrib.Size = sizeof(HIDD_ATTRIBUTES);
    ret = HidD_GetAttributes(handle, &attrib);
    if (!ret || (VID > 0 && attrib.VendorID != VID) ||
        (PID > 0 && attrib.ProductID != PID) ||
        !HidD_GetPreparsedData(handle, &hid_data)) {
      CloseHandle(handle);
      continue;
    }

    HIDP_CAPS capabilities;
    if (!HidP_GetCaps(hid_data, &capabilities) ||
        (usage_page > 0 && capabilities.UsagePage != usage_page) ||
        (usage > 0 && capabilities.Usage != usage)) {
      HidD_FreePreparsedData(hid_data);
      CloseHandle(handle);
      continue;
    }
    HidD_FreePreparsedData(hid_data);

    hid_t *hid = (struct hid_struct *)malloc(sizeof(struct hid_struct));
    if (!hid) {
      CloseHandle(handle);
      continue;
    }

    hid->handle = handle;
    hid->VersionNumber = attrib.VersionNumber;
    hid->open = 1;
    add_hid(hid);
    count++;
    if (count >= max) { return count; }
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
  if (!hid || !hid->open) return;
  hid_close(hid);
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

hid_t *hid_device::get_hid(int num) {
  hid_t *hid;
  for (hid = first_hid_; hid && num > 0; hid = hid->next, num--) ;
  return hid;
}

void hid_device::free_all_hid() {
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
  first_hid_ = last_hid_ = NULL;
}

void hid_device::hid_close(hid_t *hid) {
  CloseHandle(hid->handle);
  hid->handle = NULL;
}

bool hid_device::find_device() {
  if (first_hid_) { free_all_hid(); }
  if (!rx_event_) {
    rx_event_ = CreateEvent(nullptr, true, true, nullptr);
    tx_event_ = CreateEvent(nullptr, true, true, nullptr);
    InitializeCriticalSection(&rx_mutex_);
    InitializeCriticalSection(&tx_mutex_);
  }

  GUID id;
  HidD_GetHidGuid(&id);
  HDEVINFO info = SetupDiGetClassDevs(&id,
      nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
  if (info == INVALID_HANDLE_VALUE) { return 0; }

  SP_DEVICE_INTERFACE_DATA iface;
  int count = 0;
  for (DWORD index = 0; ; index++) {
    iface.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
    BOOL ret = SetupDiEnumDeviceInterfaces(info,
        nullptr, &id, index, &iface);
    if (!ret) { return count; }

    DWORD reqd_size;
    SetupDiGetInterfaceDeviceDetail(info,
        &iface, nullptr, 0, &reqd_size, nullptr);

    SP_DEVICE_INTERFACE_DETAIL_DATA *details =
      (SP_DEVICE_INTERFACE_DETAIL_DATA *)malloc(reqd_size);   // NOLINT
    if (details == nullptr) { continue; }

    memset(details, 0, reqd_size);
    details->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
    ret = SetupDiGetDeviceInterfaceDetail(info,
        &iface, details, reqd_size, nullptr, nullptr);
    if (!ret) {
      free(details);
      continue;
    }

    HANDLE handle = CreateFile(details->DevicePath,
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr, OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED, nullptr);
    free(details);
    if (handle == INVALID_HANDLE_VALUE) { continue; }

    HIDD_ATTRIBUTES attrib;
    PHIDP_PREPARSED_DATA hid_data;
    attrib.Size = sizeof(HIDD_ATTRIBUTES);
    ret = HidD_GetAttributes(handle, &attrib);
    if (ret && (VID > 0 && attrib.VendorID == VID) &&
        (PID > 0 && attrib.ProductID == PID) &&
        HidD_GetPreparsedData(handle, &hid_data)) {
      return true;
    }
  }

  // CloseHandle(handle);
  return false;
}

int hid_device::get_version_number() {
  if (!first_hid_) { return -1; }

  return first_hid_->VersionNumber;
}

void hid_device::droped() {
  free_all_hid();
}

} // namespace hid

MYNTEYE_END_NAMESPACE
