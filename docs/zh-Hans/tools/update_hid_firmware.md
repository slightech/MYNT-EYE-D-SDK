# 升级 HID 设备固件 {#update_hid_firmware}

## 1. 获取固件

[百度云盘](https://pan.baidu.com/s/1JLqtb7SM45HCs_kQ2VDt7Q)

[Google Drive](https://drive.google.com/open?id=1fdW7SRllBc_STAOH14q5qA_yTgexaGdx)

## 2. 编译 SDK 工具

```
cd <MYNT-EYE-D-SDK>
make tools
```

## 2.1 升级固件

```
./tools/_output/bin/writer/device_hid_update <firmware-file-path>
```
