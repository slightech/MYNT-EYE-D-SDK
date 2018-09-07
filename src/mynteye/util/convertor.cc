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

int MJPEG_TO_RGB24_LIBJPEG(unsigned char* jpg, int nJpgSize,
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

MYNTEYE_END_NAMESPACE
