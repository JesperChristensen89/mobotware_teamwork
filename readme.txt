ufuncteamwork.cpp is the main file controlling the application.
teamwork.cpp is where the image processing happens.

Mainly the cpp files have been commented, and the methods documented, and not in h files.

The commands.h/cpp is not used / not yet integrated.

The plugin is run in mobotware by running the camera server and typing:

teamwork setup

into the terminal. This will forward images from the PiCam to the plugin, and let
the plugin process the image. To stop the follower robot type:

teamwork stop

The software is developed in Kdevelop, and as tabs are used as spaces, the formatting might look skewed if inspected using a different IDE or text editor.