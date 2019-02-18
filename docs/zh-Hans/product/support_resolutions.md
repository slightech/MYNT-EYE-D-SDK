# 图像分辨率支持列表 {#support_resolutions}

| L'=left rectify, L=left, R'=right rectify, R=right, D=depth | interface | color resolution | color fps | depth resolution | depth fps |
| :---------------------------------------------------------- | :-------- | :--------------- | :-------- | :--------------- | :-------- |
| L'+D | USB3.0 | 1280x720 | 60/30/20/10 | 1280x720 | 60/30/20/10 |
| L'+D | USB3.0 | 640x480 | 60/30 | 640x480 | 60/30 |
| L'+R'+D | USB3.0 | 2560x720 | 30 | 1280x720 | 30 |
| L'+R'+D | USB3.0 | 1280x480 | 60/30 | 640x480 | 60/30 |
| L+D | USB3.0 | 1280x720 | 60/30/20/10 | 1280x720 | 60/30/20/10 |
| L+D | USB3.0 | 640x480 | 60/30 | 640x480 | 60/30 |
| L+R+D | USB3.0 | 2560x720 | 30 | 1280x720 | 30 |
| L+R+D | USB3.0 | 1280x480 | 60/30 | 640x480 | 60/30 |
| L+R | USB3.0 | 2560x720 | 60/30 | not open | null |
| L'+R' | USB3.0 | 2560x720 | 60/30 | not open | null |
| D | USB3.0 | not open | null | 1280x720 | 60/30 |
| D | USB3.0 | not open | null | 640x480 | 60/30 |
| L+R | USB2.0 | 2560x720 | 5 | not open | null |
| L'+R' | USB2.0 | 2560x720 | 5 | not open | null |
| L+R | USB2.0 | 1280x480 | 15 | not open | null |
| L'+R' | USB2.0 | 1280x480 | 15 | not open | null |
| L'+D | USB2.0 | 1280x720 | 5 | 640x720 | 5 |
| L'+D | USB2.0 | 640x480 | 15 | 320x480 | 15 |
| L+D | USB2.0 | 1280x720 | 5 | 640x720 | 5 |
| L+D | USB2.0 | 640x480 | 15 | 320x480 | 15 |
| L' | USB2.0 | 1280x720 | 5 | not open | null |
| L | USB2.0 | 1280x720 | 5 | not open | null |
| D | USB2.0 | not open | null | 640x720 | 5 |
| D | USB2.0 | not open | null | 320x480 | 15 |
| L+R | USB2.0/MJPG | 2560x720 | 5 | not open | null |
| L+R | USB2.0/MJPG | 1280x480 | 15 | not open | null |
| L | USB2.0/MJPG | 1280x720 | 5 | not open | null |

> 注意：
> * L'=left rectify image, L=left image
> * R'=right rectify image, R=right image, D=depth image
> * 在IR Depth Only模式下，帧率只支持15fps和30fps.
