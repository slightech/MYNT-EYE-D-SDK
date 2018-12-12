# 升级 HID 设备固件 {#update_hid_firmware}

## 获取固件

最新固件： mynt-eye-d-hid-firmware-1.0.bin [Google Drive](https://drive.google.com/open?id=1fdW7SRllBc_STAOH14q5qA_yTgexaGdx), [百度网盘](https://pan.baidu.com/s/1JLqtb7SM45HCs_kQ2VDt7Q)

## 编译 SDK 工具

```
cd <sdk>
make tools
```

## 升级固件

```
./tools/_output/bin/writer/device_hid_update <firmware-file-path>
```
