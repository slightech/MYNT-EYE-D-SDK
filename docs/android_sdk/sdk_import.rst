SDK Install
====================
1. Get sdk resource `SDK Download <https://mynt-eye-d-sdk.readthedocs.io/en/latest/android_sdk/sdk_download.html>`__ 。
2. New android project(example: Android Studio)
3. Put the aar file of SDK to "libs" directory (app / libs)
4. Add arr support to the "build.gradle" file, like below

::

   dependencies {
       implementation fileTree(include: ['*.aar'], dir: 'libs')
       ....
   }

5. Build --> Make Project
