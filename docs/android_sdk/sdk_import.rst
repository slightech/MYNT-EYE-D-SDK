SDK Install
====================

1. Put the aar file of SDK to "libs" directory
2. Add arr support to the "build.gradle" file, like below

::

   dependencies {
       implementation fileTree(include: ['*.aar'], dir: 'libs')
       ....
   }
