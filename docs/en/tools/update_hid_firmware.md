# Update HID Firmware {#update_hid_firmware}

## Get Firmware

Latest firmware: mynteye-d-hid-firmware-1.0.bin [Google Drive](https://drive.google.com/open?id=1gAbTf6W10a8iwT7L9TceMVgxQCWKnEsx), [Baidu Pan](https://pan.baidu.com/s/1wMfrRTylDCNFqmn2mxvExw)

## Compile SDK Tools

```
cd <sdk>
make tools
```

## Update Firmware

```
./tools/_output/bin/writer/device_hid_update <firmware-file-path>
```
