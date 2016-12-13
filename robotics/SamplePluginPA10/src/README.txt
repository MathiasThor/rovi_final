..____ ___  ____  _____    ____ _   _ ___ ____  _____
./ ___/ _ \|  _ \| ____|  / ___| | | |_ _|  _ \| ____|
| |  | | | | | | |  _|   | |  _| | | || || | | |  _|
| |__| |_| | |_| | |___  | |_| | |_| || || |_| | |___
.\____\___/|____/|_____|  \____|\___/|___|____/|_____|
=============================================================================
Code guide by Mathias Thor & Christian Koed Pedersen
=============================================================================

************
* REQUIRED *
************
  OpenCV 3.1.0 compiled with the contrib libary
  RobWork

****************
* INSTALLATION *
****************
  Go to SamplePluginPA10/build

  Execute:  cmake ./..

  Open to SamplePluginPA10/src/SamplePlugin.hpp and change the path variable
  to the location of your SamplePluginPA10 folder.

  Execute: make (inside the build folder)

  Then just start RobWorkStudio and load the plug in which is located in:
  SamplePluginPA10/libs/release/libSamplePlugin.so

*********************
* PLUG IN INTERFACE *
*********************
  ROBOT CONTROL TAB:
  Start movement:   Starts and stops the robot tracking
  Follow one frame: Tracks one single frame
  Reset: Resets the Simulation back to default parameters and position

  TEXTURE LOADER TAB:
  First combobox:   Chose the background
  Second combobox:  Chose the marker type
  Load textures:    Loads the chosen textures intro RobWorkStudio
  - Note: Remember to load textures before starting the movement if using
          feature extraction with computer vision.

  MOTION & DELTA T TAB:
  Combobox:   Chose the marker motion type (slow, medium or fast)
  Spinbox:    Alter the delta T used for limiting the velocity of the joints
  - Note: This spinbox can only be used as input and do not display the
          current delta T if it is altered inside the program.

  The Test Run button start a test run and stores the data inside the folder:
  SamplePluginPA10/data

********************************
* CHANGING THE TRACKING METHOD *
********************************
  This section describes how to change the tracking method.
  It is only possible to do this in the SamplePlugin.hpp file.

  TRACKING METHOD USING THE MARKER FRAME:

  TRACKING METHOD USING FEATURE EXTRACTION:

********************
* CONTENT OF FILES *
********************

  src/SamplePlugin.cpp:

  src/SamplePlugin.hpp:

  src/vision_source_files/color_detector.cpp:

  src/vision_header_files/color_detector.hpp:

  ../vision FORTSÆT HERFRA
