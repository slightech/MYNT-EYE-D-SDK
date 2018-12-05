# Set open parameters {#set_open_params}

## Set the resolution of image

Using the `params.stream_mode` parameter, you can set the resolution of the image.

> Attention: Now image resolution supports 4 types: 640X480，1280x720 for single camera. 1280x480， 2560x720 for left and right camera.

Reference code snippet:

```bash
// Stream mode: left color only
// params.stream_mode = StreamMode::STREAM_640x480;  // vga
// params.stream_mode = StreamMode::STREAM_1280x720;  // hd
// Stream mode: left+right color
// params.stream_mode = StreamMode::STREAM_1280x480;  // vga
params.stream_mode = StreamMode::STREAM_2560x720;  // hd
```

## Set the frame rate of image

Using the `params.framerate` parameter, you can set the frame rate of image.

> Attention
> - The effective fps of the image(0-60)
> - The effective fps of the image in 2560x720 resolution (0-30)

Reference code snippet:

```bash
// Framerate: 10(default), [0,60], [0,30](STREAM_2560x720)
params.framerate = 10;
```

## Set color mode

Using the `params.color_mode` parameter，you can set the color mode of image.

`COLOR_RAW` is original image， `COLOR_RECTIFIED` is rectified image.

Reference code snippet:

```bash
// Color mode: raw(default), rectified
// params.color_mode = ColorMode::COLOR_RECTIFIED;
```

## Set depth mode

Using the `params.depth_mode` parameter，you can set the depth mode.

`DEPTH_COLORFUL` is colorful depth image，`DEPTH_GRAY` is grey depth image， `DEPTH_RAW` is original depth image。

Reference code snippet:

```bash
// Depth mode: colorful(default), gray, raw
// params.depth_mode = DepthMode::DEPTH_GRAY;
```

## Enable auto exposure and auto white balance

Set `params.state_ae` and `params.state_awb` to `true` , you can enable auto exposure and auto white balance.

By default auto exposure and auto white balance are enabled，if you want to disable，you can set parameters to `false` .

Reference code snippet:

```bash
// Auto-exposure: true(default), false
// params.state_ae = false;

// Auto-white balance: true(default), false
// params.state_awb = false;
```

## Enable IR and its adjustments function

Using the `params.ir_intensity` parameter，you can set IR’s intensity of image.
Enabling IR is setting `params.ir_intensity` greater than 0. The greater the value, the greater the IR’s intensity.(max is 10).

Reference code snippet:

```bash
// Infrared intensity: 0(default), [0,10]
params.ir_intensity = 4;
```

## Enable IR Interleave

Using the `params.ir_interleave` parameter，you can set IR Interleave function. This is enabled by default.

> Attention
> - This function doesn't work on 30 frame rate.

Reference code snippet:

```bash
// IR Interleave: true(default), false
// Note: When frame rate less than 30fps, ir interleave will be not available.
// params.ir_interleave = false;
```

Reference running results on Linux:

```
Open device: 0, /dev/video1

D/eSPDI_API: SetPropertyValue control=7 value=0D/eSPDI_API: SetPropertyValue control=7 value=35D/eSPDI_API: SetPropertyValue control=7 value=1-- Auto-exposure state: enabled
D/eSPDI_API: SetPropertyValue control=7 value=0D/eSPDI_API: SetPropertyValue control=7 value=12D/eSPDI_API: SetPropertyValue control=7 value=1-- Auto-white balance state: enabled
-- Framerate: 5
D/eSPDI_API: SetPropertyValue control=7 value=4 SetDepthDataType: 4
-- Color Stream: 1280x720 YUYV
-- Depth Stream: 1280x720 YUYV

WARNING:: IR interleave will not be effective for framerate less than 30 fps!

D/eSPDI_API: SetPropertyValue control=7 value=0D/eSPDI_API: SetPropertyValue control=7 value=3D/eSPDI_API: SetPropertyValue control=7 value=4
-- IR intensity: 4
D/eSPDI_API: CVideoDevice::OpenDevice 1280x720 fps=5

Open device success
```

Complete code samples，see [get_image](https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_image.cc) .
