# Support Resolutions {#support_resolutions}

| L'=left rectify, L=left, R'=right rectify, R=right, D=depth | interface | color resolution | color fps | depth resolution | depth fps |
| :---------------------------------------------------------- | :-------- | :--------------- | :-------- | :--------------- | :-------- |
| L'+D | USB3.0 | 1280x720 | 60/30/20/10 | 1280x720 | 60/30/20/10 |
| L'+D | USB3.0 | 640x480 | 60/30 | 640x480 | 60/30 |
| L'+R'+D | USB3.0 | 2560x720 | 30 | 1280x720 | 30 |
| L'+R'+D | USB3.0 | 1280x480 | 60/30 | 1280x480 | 60/30 |
| L+D | USB3.0 | 1280x720 | 60/30/20/10 | 1280x720 | 60/30/20/10 |
| L+D | USB3.0 | 640x480 | 60/30 | 640x480 | 60/30 |
| L+R+D | USB3.0 | 2560x720 | 30 | 1280x720 | 30 |
| L+R+D | USB3.0 | 1280x480 | 60/30 | 1280x480 | 60/30 |
| L+R | USB3.0 | 2560x720 | 60/30 | not open | null |
| L'+R' | USB3.0 | 2560x720 | 60/30 | not open | null |
| D | USB3.0 | not open | null | 1280x720 | 60/30 |
| D | USB3.0 | not open | null | 640x780 | 60/30 |
| L'+D | USB2.0 | 1280x720 | 5 | 640x720 | 5 |
| L'+D | USB2.0 | 640x480 | 5 | 320x480 | 5 |
| L+D | USB2.0 | 1280x720 | 5 | 640x720 | 5 |
| L+D | USB2.0 | 640x480 | 5 | 320x480 | 5 |
| L' | USB2.0 | 1280x720 | 10 | not open | null |
| L | USB2.0 | 1280x720 | 10 | not open | null |


Note:

* When resolution 1280x720, color and depth maximum support 60fps.
* When resolution 640x480, color and depth maximum support 60fps.
* When resolution 1280x480, color and depth only support 60, 30fps.
* When resolution 2560x720, color and depth only support 30fps.
* When only the color device be opened，2560x720 only support 60, 30fps.
* In IR Depth Only mode, frame rate is supported between 15fps and 30fps.
