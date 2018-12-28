# How to use SDK with Qt Creator {#qtcreator}

This tutorial will create a Qt project with Qt Creator to start using SDK.

> You could find the project demo in `<sdk>/platforms/projects/qtcreator` directory.

## Preparation

* Windows: install the win pack of SDK
* Linux: build from source and `make install`

## Create Project

Open Qt Creator, then `New Project`,

![](images/projects/qtcreator/new_project.png)

\latexonly
\includegraphics[width=1\textwidth]{new_project.png}
\endlatexonly

Choose `Qt Widgets Application`,

![](images/projects/qtcreator/new_project2.png)

\latexonly
\includegraphics[width=1\textwidth]{new_project2.png}
\endlatexonly

Set project location and its name,

![](images/projects/qtcreator/new_project3.png)

\latexonly
\includegraphics[width=1\textwidth]{new_project3.png}
\endlatexonly

Select the build kits,

![](images/projects/qtcreator/new_project4.png)

\latexonly
\includegraphics[width=1\textwidth]{new_project4.png}
\endlatexonly

Then, it will generate the skeleton source files,

![](images/projects/qtcreator/new_project5.png)

\latexonly
\includegraphics[width=1\textwidth]{new_project5.png}
\endlatexonly

![](images/projects/qtcreator/new_project6.png)

\latexonly
\includegraphics[width=1\textwidth]{new_project6.png}
\endlatexonly

Finally, you will see the new project like this,

![](images/projects/qtcreator/new_project7.png)

\latexonly
\includegraphics[width=1\textwidth]{new_project7.png}
\endlatexonly

## Config Project

Edit `mynteyed_demo.pro` to add `INCLUDEPATH` and `LIBS`.

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

## Start using SDK

Include the headers of SDK and start using its APIs, could see the project demo.

### Windows

Should select "Release" to run the project.

![](images/projects/qtcreator/release_run.png)

\latexonly
\includegraphics[width=0.5\textwidth]{release_run.png}
\endlatexonly

Then you will see the main window,

![](images/projects/qtcreator/run_win.png)

\latexonly
\includegraphics[width=1\textwidth]{run_win.png}
\endlatexonly

### Linux

Run the project and you will see the main window,

![](images/projects/qtcreator/run_linux.png)

\latexonly
\includegraphics[width=1\textwidth]{run_linux.png}
\endlatexonly
