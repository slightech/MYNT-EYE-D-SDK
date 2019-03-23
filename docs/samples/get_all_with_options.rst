.. _get_all_with_options:

Get different types of image by options
=======================================

``get_all_with_options`` sample can add different options to control
device.

``get_all_with_options -h``:

.. code-block:: bash

   Open device with different options.

   Options:
     -h, --help            show this help message and exit
     -m, --imu             Enable imu datas

     Open Params:
       The open params

       -i INDEX, --index=INDEX
                           Device index
       -f RATE, --rate=RATE
                           Framerate, range [0,60], [30](STREAM_2560x720),
                           default: 10
       --dev-mode=MODE       Device mode, default 2 (DEVICE_ALL)
                             0: DEVICE_COLOR, left y right - depth n
                             1: DEVICE_DEPTH, left n right n depth y
                             2: DEVICE_ALL,   left y right - depth y
                             Note: y: available, n: unavailable, -: depends on
                           stream mode
       --cm=MODE           Color mode, default 0 (COLOR_RAW)
                             0: COLOR_RAW, color raw
                             1: COLOR_RECTIFIED, color rectified
       --dm=MODE           Depth mode, default 2 (DEPTH_COLORFUL)
                             0: DEPTH_RAW
                             1: DEPTH_GRAY
                             2: DEPTH_COLORFUL
       --sm=MODE           Stream mode of color & depth,
                           default 2 (STREAM_1280x720)
                             0: STREAM_640x480, 480p, vga, left
                             1: STREAM_1280x480, 480p, vga, left+right
                             2: STREAM_1280x720, 720p, hd, left
                             3: STREAM_2560x720, 720p, hd, left+right
       --csf=MODE          Stream format of color,
                           default 1 (STREAM_YUYV)
                             0: STREAM_MJPG
                             1: STREAM_YUYV
       --dsf=MODE          Stream format of depth,
                           default 1 (STREAM_YUYV)
                             1: STREAM_YUYV
       --ae                Enable auto-exposure
       --awb               Enable auto-white balance
       --ir=VALUE          IR intensity, range [0,6], default 0
       --ir-depth          Enable ir-depth-only

     Feature Toggles:
       The feature toggles

       --proc=MODE         Enable process mode, e.g. imu assembly, temp_drift
                             0: PROC_NONE
                             1: PROC_IMU_ASSEMBLY
                             2: PROC_IMU_TEMP_DRIFT
                             3: PROC_IMU_ALL
       --img-info          Enable image info, and sync with image

e.g. \ ``./samples/_output/bin/get_all_with_options -f 60 --dev-mode=0 --sm=2``
displays 1280x720 60fps left unrectified image.

Complete code samples，see
`get_all_with_options.cc <https://github.com/slightech/MYNT-EYE-D-SDK/blob/master/samples/src/get_all_with_options.cc>`__
.
