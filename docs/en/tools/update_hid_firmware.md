# Update HID Firmware {#update_hid_firmware}

## Get Firmware

Latest firmware: mynt-eye-d-hid-firmware-1.0.bin [Google Drive](https://drive.google.com/open?id=1fdW7SRllBc_STAOH14q5qA_yTgexaGdx), [Baidu Pan](https://pan.baidu.com/s/1JLqtb7SM45HCs_kQ2VDt7Q)

## Compile SDK Tools

```
cd <sdk>
make tools
```

## Update Firmware

```
./tools/_output/bin/writer/device_hid_update <firmware-file-path>
```
