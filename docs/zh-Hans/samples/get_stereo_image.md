# 获取双目图像 {#get_stereo_image}

API 通过 `DeviceMode::DEVICE_COLOR` 参数获取图像数据，或者 `DeviceMode::DEVICE_ALL` 同时捕获图像和深度数据。

通过 `GetStreamData()` 函数，就能获取想要的数据。

参考代码片段：

```
// Device mode, default DEVICE_ALL
//   DEVICE_COLOR: IMAGE_LEFT_COLOR y IMAGE_RIGHT_COLOR - IMAGE_DEPTH n
//   DEVICE_DEPTH: IMAGE_LEFT_COLOR n IMAGE_RIGHT_COLOR n IMAGE_DEPTH y
//   DEVICE_ALL:   IMAGE_LEFT_COLOR y IMAGE_RIGHT_COLOR - IMAGE_DEPTH y
// Note: y: available, n: unavailable, -: depends on #stream_mode
params.dev_mode = DeviceMode::DEVICE_DEPTH;


auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR);
    if (left_color.img) {
    cv::Mat left = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
    painter.DrawSize(left, CVPainter::TOP_LEFT);
    painter.DrawStreamData(left, left_color, CVPainter::TOP_RIGHT);
    painter.DrawInformation(left, util::to_string(counter.fps()),
        CVPainter::BOTTOM_RIGHT);
    cv::imshow("left color", left);
```

完整代码样例，请见[get_stereo_image.cc](https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_stereo_image.cc) 。
