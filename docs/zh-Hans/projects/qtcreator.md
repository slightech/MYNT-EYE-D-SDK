# Qt Creator 如何使用 SDK {#qtcreator}

该教程将会使用 Qt creator 创建 Qt 项目来运行 SDK 。

> 你可以在 `<sdk>/platforms/projects/qtcreator` 目录下找到工程样例。

## 准备

* Windows: 安装 SDK 的 exe 包
* Linux: 使用源代码编译和 `make install`

## 创建项目

打开 Qt Creator ，然后 `New Project`，

![](images/projects/qtcreator/new_project.png)

选择 `Qt Widgets Application` ，

![](images/projects/qtcreator/new_project2.png)

设置项目位置和名字，

![](images/projects/qtcreator/new_project3.png)

选择 build kits ，

![](images/projects/qtcreator/new_project4.png)

然后他将会生成框架源文件，

![](images/projects/qtcreator/new_project5.png)

![](images/projects/qtcreator/new_project6.png)

最后，你将会看到这样的新项目工程，

![](images/projects/qtcreator/new_project7.png)

## 配置项目

添加 `INCLUDEPATH` 和 `LIBS` 到 `mynteyed_demo.pro` 。

```bash
win32 {
    SDK_ROOT = "$$(MYNTEYED_SDK_ROOT)"
    isEmpty(SDK_ROOT) {
        error( "MYNTEYED_SDK_ROOT not found, please install SDK firstly" )
    }
    message("SDK_ROOT: $$SDK_ROOT")

    INCLUDEPATH += "$$SDK_ROOT/include"
    LIBS += "$$SDK_ROOT/lib/mynteye_depth.lib"
}

unix {
    INCLUDEPATH += /usr/local/include
    LIBS += -L/usr/local/lib -lmynteye_depth
}
```

## 使用SDK

可以参考工程样例添加头文件和使用 API 。

### Windows

选择 "Release" 来运行项目。

![](images/projects/qtcreator/release_run.png)

然后你将看到主窗口，

![](images/projects/qtcreator/run_win.png)

### Linux

运行项目，你将看到主窗口，

![](images/projects/qtcreator/run_linux.png)
