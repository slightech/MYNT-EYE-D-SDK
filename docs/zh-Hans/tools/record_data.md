# 录制数据集 {#record_data}

SDK 提供了录制数据集的工具 record. 工具的详细信息见 tools/README.md

Linux 系统运行命令:

```
./tools/_output/bin/dataset/record
```

Windows 系统运行命令:

```
.\tools\_output\bin\dataset\record.bat
```

Linux 系统上的结果参考:

```
$ ./tools/_output/bin/dataset/record
Saved 1007 imgs, 20040 imus to ./dataset
I0513 21:29:38.608772 11487 record.cc:118] Time beg: 2018-05-13 21:28:58.255395, end: 2018-05-13 21:29:38.578696, cost: 40323.3ms
I0513 21:29:38.608853 11487 record.cc:121] Img count: 1007, fps: 24.9732
I0513 21:29:38.608873 11487 record.cc:123] Imu count: 20040, hz: 496.983
```

结果默认保存在 `<workdir>/dataset` 中. 您也可以使用参数指定自定义目录存放结果.

录制结果目录详情:

```
<workdir>/
└─dataset/
   ├─left/
   │  ├─stream.txt  # Image infomation
   │  ├─000000.png  # Image，index 0
   │  └─...
   ├─right/
   │  ├─stream.txt  # Image information
   │  ├─000000.png  # Image，index 0
   │  └─...
   └─motion.txt  # IMU information
```

Copyright 2018. MYNTEYE
