# Recording data sets
The SDK provides the tool record for recording data sets. Tool details can be seen in tools/README.md .

Reference run command on Linux:

```
./tools/_output/bin/dataset/record
```
Reference run command on Windows:

```
.\tools\_output\bin\dataset\record.bat
```

Results save into <workdir>/dataset by default. You can also add parameter, select other directory to save.

Record contents:

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
