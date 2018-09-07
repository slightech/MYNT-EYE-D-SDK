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
#include "mynteye/util/convertor.h"

#include "mynteye/util/log.h"

MYNTEYE_BEGIN_NAMESPACE

namespace {

METHODDEF(void)
my_error_exit(j_common_ptr cinfo) {
  /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
  my_error_ptr myerr = (my_error_ptr) cinfo->err;

  /* Always display the message. */
  /* We could postpone this until after returning, if we chose. */
  (*cinfo->err->output_message) (cinfo);

  /* Return control to the setjmp point */
  longjmp(myerr->setjmp_buffer, 1);
}

}  // namespace

int MJPEG_TO_RGB_LIBJPEG(unsigned char* jpg, int nJpgSize,
    unsigned char* rgb) {
  struct jpeg_decompress_struct cinfo;
  struct my_error_mgr jerr;

  int rc;
  int row_stride, width, height, pixel_size;

  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.error_exit = my_error_exit;

  if (setjmp(jerr.setjmp_buffer)) {
    jpeg_destroy_decompress(&cinfo);
    return 0;
  }

  jpeg_create_decompress(&cinfo);
  jpeg_mem_src(&cinfo, jpg, nJpgSize);

  rc = jpeg_read_header(&cinfo, TRUE);

  if (rc != 1) {
    LOGE("Error: File does not seem to be a normal JPEG !!");
  }

  jpeg_start_decompress(&cinfo);

  width = cinfo.output_width;
  height = cinfo.output_height;
  pixel_size = cinfo.output_components;

  row_stride = width * pixel_size;

  while (cinfo.output_scanline < cinfo.output_height) {
    unsigned char *buffer_array[1];
    // buffer_array[0] = rgb + (width * height * 3) -
    //     (cinfo.output_scanline) * row_stride;
    buffer_array[0] = rgb + (cinfo.output_scanline) * row_stride;

    jpeg_read_scanlines(&cinfo, buffer_array, 1);
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);

  UNUSED(height);
  return 0;
}

namespace {

int yuv_to_rgb_pixel(int y, int u, int v) {
  unsigned int pixel32 = 0;
  unsigned char *pixel = (unsigned char *)&pixel32;
  int r, g, b;

  r = y + (1.370705 * (v-128));
  g = y - (0.698001 * (v-128)) - (0.337633 * (u-128));
  b = y + (1.732446 * (u-128));

  if (r > 255) r = 255;
  if (g > 255) g = 255;
  if (b > 255) b = 255;

  if (r < 0) r = 0;
  if (g < 0) g = 0;
  if (b < 0) b = 0;

  pixel[0] = r * 220 / 256;
  pixel[1] = g * 220 / 256;
  pixel[2] = b * 220 / 256;

  return pixel32;
}

}  // namespace

int YUYV_TO_RGB(unsigned char* yuv, unsigned char* rgb, unsigned int width,
    unsigned int height) {
  unsigned int in, out = 0;
  unsigned int pixel_16;
  unsigned char pixel_24[3];
  unsigned int pixel32;
  int y0, u, y1, v;
  for (in = 0; in < width * height * 2; in += 4) {
    pixel_16 =
    yuv[in + 3] << 24 |
    yuv[in + 2] << 16 |
    yuv[in + 1] <<  8 |
    yuv[in + 0];

    y0 = (pixel_16 & 0x000000ff);
    u  = (pixel_16 & 0x0000ff00) >>  8;
    y1 = (pixel_16 & 0x00ff0000) >> 16;
    v  = (pixel_16 & 0xff000000) >> 24;

    pixel32 = yuv_to_rgb_pixel(y0, u, v);

    pixel_24[0] = (pixel32 & 0x000000ff);
    pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
    pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
    rgb[out++] = pixel_24[0];
    rgb[out++] = pixel_24[1];
    rgb[out++] = pixel_24[2];

    pixel32 = yuv_to_rgb_pixel(y1, u, v);

    pixel_24[0] = (pixel32 & 0x000000ff);
    pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
    pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
    rgb[out++] = pixel_24[0];
    rgb[out++] = pixel_24[1];
    rgb[out++] = pixel_24[2];
  }

  return 0;
}

int YUYV_TO_BGR(unsigned char* yuv, unsigned char* bgr, unsigned int width,
    unsigned int height) {
  unsigned int in, out = 0;
  unsigned int pixel_16;
  unsigned char pixel_24[3];
  unsigned int pixel32;
  int y0, u, y1, v;
  for (in = 0; in < width * height * 2; in += 4) {
    pixel_16 =
    yuv[in + 3] << 24 |
    yuv[in + 2] << 16 |
    yuv[in + 1] <<  8 |
    yuv[in + 0];

    y0 = (pixel_16 & 0x000000ff);
    u  = (pixel_16 & 0x0000ff00) >>  8;
    y1 = (pixel_16 & 0x00ff0000) >> 16;
    v  = (pixel_16 & 0xff000000) >> 24;

    pixel32 = yuv_to_rgb_pixel(y0, u, v);

    pixel_24[0] = (pixel32 & 0x000000ff);
    pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
    pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
    bgr[out++] = pixel_24[2];
    bgr[out++] = pixel_24[1];
    bgr[out++] = pixel_24[0];

    pixel32 = yuv_to_rgb_pixel(y1, u, v);

    pixel_24[0] = (pixel32 & 0x000000ff);
    pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
    pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
    bgr[out++] = pixel_24[2];
    bgr[out++] = pixel_24[1];
    bgr[out++] = pixel_24[0];
  }

  return 0;
}

namespace {

void reverse(unsigned char* rgb, unsigned int width, unsigned int height) {
  unsigned char tmp;
  for (unsigned int i = 0, n = width * height; i < n; i++) {
    tmp = *rgb;         // tmp = r
    *rgb = *(rgb + 2);  // r = b
    *(rgb + 2) = tmp;   // b = tmp
    rgb += 3;
  }
}

}  // namespace

void RGB_TO_BGR(unsigned char* rgb,
    unsigned int width, unsigned int height) {
  reverse(rgb, width, height);
}

void BGR_TO_RGB(unsigned char* bgr,
    unsigned int width, unsigned int height) {
  reverse(bgr, width, height);
}

MYNTEYE_END_NAMESPACE
