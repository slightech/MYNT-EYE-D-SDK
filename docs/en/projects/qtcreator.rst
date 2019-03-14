.. role:: raw-latex(raw)
   :format: latex
..

.. _qtcreator:

How to use SDK with Qt Creator
==============================

This tutorial will create a Qt project with Qt Creator to start using
SDK.

   You could find the project demo in
   ``<sdk>/platforms/projects/qtcreator`` directory.

Preparation
-----------

-  Windows: install the win pack of SDK
-  Linux: build from source and ``make install``

Create Project
--------------

Open Qt Creator, then ``New Project``,

|image0|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=1\textwidth]{new_project.png}`
:raw-latex:`\endlatexonly`

Choose ``Qt Widgets Application``,

|image1|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=1\textwidth]{new_project2.png}`
:raw-latex:`\endlatexonly`

Set project location and its name,

|image2|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=1\textwidth]{new_project3.png}`
:raw-latex:`\endlatexonly`

Select the build kits,

|image3|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=1\textwidth]{new_project4.png}`
:raw-latex:`\endlatexonly`

Then, it will generate the skeleton source files,

|image4|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=1\textwidth]{new_project5.png}`
:raw-latex:`\endlatexonly`

|image5|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=1\textwidth]{new_project6.png}`
:raw-latex:`\endlatexonly`

Finally, you will see the new project like this,

|image6|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=1\textwidth]{new_project7.png}`
:raw-latex:`\endlatexonly`

Config Project
--------------

Edit ``mynteyed_demo.pro`` to add ``INCLUDEPATH`` and ``LIBS``.

.. code-block:: bash

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

Start using SDK
---------------

Include the headers of SDK and start using its APIs, could see the
project demo.

Windows
~~~~~~~

Should select “Release” to run the project.

|image7|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=0.5\textwidth]{release_run.png}`
:raw-latex:`\endlatexonly`

Then you will see the main window,

|image8|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=1\textwidth]{run_win.png}`
:raw-latex:`\endlatexonly`

Linux
~~~~~

Run the project and you will see the main window,

|image9|

.. raw:: latex

   \latexonly

:raw-latex:`\includegraphics[width=1\textwidth]{run_linux.png}`
:raw-latex:`\endlatexonly`

.. |image0| image:: ../../static/images/projects/qtcreator/new_project.png
.. |image1| image:: ../../static/images/projects/qtcreator/new_project2.png
.. |image2| image:: ../../static/images/projects/qtcreator/new_project3.png
.. |image3| image:: ../../static/images/projects/qtcreator/new_project4.png
.. |image4| image:: ../../static/images/projects/qtcreator/new_project5.png
.. |image5| image:: ../../static/images/projects/qtcreator/new_project6.png
.. |image6| image:: ../../static/images/projects/qtcreator/new_project7.png
.. |image7| image:: ../../static/images/projects/qtcreator/release_run.png
.. |image8| image:: ../../static/images/projects/qtcreator/run_win.png
.. |image9| image:: ../../static/images/projects/qtcreator/run_linux.png
