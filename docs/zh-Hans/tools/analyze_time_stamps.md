# 分析时间戳 {#analyze_time_stamps}

SDK 提供了时间戳分析工具 stamp_analytics.py. 工具的详细信息见 tools/README.

Linux 系统运行命令:

```
$ python tools/analytics/stamp_analytics.py -i dataset -c tools/config/mynteye/mynteye_config.yaml
```

Linux 系统上的结果参考:

```
$ python tools/analytics/stamp_analytics.py -i dataset -c tools/config/mynteye/mynteye_config.yaml
stamp analytics ...
  input: dataset
  outdir: dataset
open dataset ...
save to binary files ...
  binimg: dataset/stamp_analytics_img.bin
  binimu: dataset/stamp_analytics_imu.bin
  img: 1007, imu: 20040

rate (Hz)
  img: 25, imu: 500
sample period (s)
  img: 0.04, imu: 0.002

diff count
  imgs: 1007, imus: 20040
  imgs_t_diff: 1006, imus_t_diff: 20039

diff where (factor=0.1)
  imgs where diff > 0.04*1.1 (0)
  imgs where diff < 0.04*0.9 (0)
  imus where diff > 0.002*1.1 (0)
  imus where diff < 0.002*0.9 (0)

image timestamp duplicates: 0

save figure to:
  dataset/stamp_analytics.png
stamp analytics done
```

分析结果图保存在 dataset 目录中. 如下:

![stamp analytics](stamp_analytics.png)

\latexonly
\includegraphics[width=1\textwidth]{stamp_analytics.png}
\endlatexonly

另外，可以使用 -h 参数查看工具详细参数选项.

```
$ python tools/analytics/stamp_analytics.py -h
```



Copyright 2018. MYNTEYE




