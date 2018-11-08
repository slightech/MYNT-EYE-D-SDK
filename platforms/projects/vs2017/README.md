# How to use MYNT® EYE Depth SDK with Visual Studio 2017

This tutorial will create a project with Visual Studio 2017 to start using MYNT® EYE Depth SDK.

## Preparation

Install the win pack of MYNT® EYE Depth SDK.

## Create Project

Open Visual Studio 2017, then `File > New > Project`,

![](images/1_new_pro.png)

Select "Windows Console Application", set the project's name and location, click "OK",

![](images/2_new_pro.png)

Finally, you will see the new project like this,

![](images/3_new_pro.png)

## Config Properties

Right click the project, and open its "Properties" window,

![](images/4_config.png)

Change "Configuration" to "All Configurations", then add the following paths to "Additional Include Directories",

```bash
$(MYNTEYED_SDK_ROOT)\include
$(MYNTEYED_SDK_ROOT)\3rdparty\opencv\build\include
```

![](images/5_config_include.png)

Add the following definitions to "Preprocessor Definitions",

```bash
WITH_OPENCV
WITH_OPENCV3
```

![](images/6_config_definition.png)

Add the following paths to "Additional Library Directories",

```bash
$(MYNTEYED_SDK_ROOT)\lib
$(MYNTEYED_SDK_ROOT)\3rdparty\opencv\build\x64\vc15\lib
```

![](images/7_config_lib_dir.png)

Add the following libs to "Additional Dependencies",

```bash
mynteye_depth.lib
opencv_world343.lib
```

![](images/8_config_lib.png)

## Start using SDK

Include the headers of SDK and start using its APIs,

![](images/9_run_x64.png)

Select "Release x64" to run the project.
