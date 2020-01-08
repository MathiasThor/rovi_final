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
  Go to robotics/SamplePluginPA10/build

  Execute:  cmake ..

  Open to SamplePluginPA10/src/SamplePlugin.hpp and change the path variable
  to the location of your SamplePluginPA10 folder.

  Execute: make (inside the build folder)

  Then just start RobWorkStudio and load the plug in which is located in:
  robotics/SamplePluginPA10/libs/release/libSamplePlugin.so

  ----
  Note: To install the vision part only go to "vision/build" and "cmake .."
        and then execute "make"

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
  robotics/SamplePluginPA10/data

********************************
* CHANGING THE TRACKING METHOD *
********************************
  This section describes how to change the tracking method.
  It is only possible to do this in the SamplePlugin.hpp file.

  TRACKING METHOD USING THE MARKER FRAME:
    In the "robotics/SamplePluginPA10/src/SamplePlugin.hpp" set:
      bool cvOrFile = false;
      comment in the points you want to track (that is PT0, PT1, PT3)
      comment out the other set of point
        If original points are chosen, then set double kappa = 1.2;

  TRACKING METHOD USING FEATURE EXTRACTION:
    In the "robotics/SamplePluginPA10/src/SamplePlugin.hpp" set:
      bool cvOrFile = true;
      int cv_choice = 1; (to follow the color marker)
      double kappa  = 3;
      REMEMBER TO LOAD THE RIGHT MARKER TEXTURE IN THE SIMULATION

  CHANGE NUMBER OF POINTS TO BE TRACKED:
    In the "robotics/SamplePluginPA10/src/SamplePlugin.hpp" set:
      int numOfPoints = 1; (Tracks the middle point of the marker or from CV)
      int numOfPoints = 3; (Tracks three points from CV or PT0, PT1, PT2)

********************
* CONTENT OF FILES *
********************
All of the C++ files are well commented, so this list will only
give a brief overview of whet the purpose of the files is.
---

  robotics/SamplePluginPA10/src/SamplePlugin.cpp:
    Includes most of the code for the sample plug in
    Includes the code used to control the robot (algorithm 1)
    Includes the code used for testing the system

  robotics/SamplePluginPA10/src/vision_source_files/color_detector.cpp:

  robotics/SamplePluginPA10/src/SamplePlugin.ui:
    Includes the code used to present the plug in visually.

  vision/source_files/main.cpp

  vision/source_files/general_functions.cpp

  vision/source_files/color_detector.cpp:

  vision/source_files/corny_detector.cpp
  
  vision/header_files/*.hpp:
    Contains all the header files for the vision code.
