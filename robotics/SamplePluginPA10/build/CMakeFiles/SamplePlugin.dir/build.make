# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build

# Include any dependencies generated for this target.
include CMakeFiles/SamplePlugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SamplePlugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SamplePlugin.dir/flags.make

ui_SamplePlugin.h: ../src/SamplePlugin.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ui_SamplePlugin.h"
	/usr/lib/x86_64-linux-gnu/qt4/bin/uic -o /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/ui_SamplePlugin.h /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/src/SamplePlugin.ui

src/moc_SamplePlugin.cxx: ../src/SamplePlugin.hpp
src/moc_SamplePlugin.cxx: src/moc_SamplePlugin.cxx_parameters
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating src/moc_SamplePlugin.cxx"
	cd /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/src && /usr/lib/x86_64-linux-gnu/qt4/bin/moc @/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/src/moc_SamplePlugin.cxx_parameters

qrc_resources.cxx: ../src/pa_icon.png
qrc_resources.cxx: src/resources.qrc.depends
qrc_resources.cxx: ../src/resources.qrc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating qrc_resources.cxx"
	/usr/lib/x86_64-linux-gnu/qt4/bin/rcc -name resources -o /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/qrc_resources.cxx /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/src/resources.qrc

CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o: CMakeFiles/SamplePlugin.dir/flags.make
CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o: ../src/SamplePlugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o -c /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/src/SamplePlugin.cpp

CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/src/SamplePlugin.cpp > CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.i

CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/src/SamplePlugin.cpp -o CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.s

CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o.requires:

.PHONY : CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o.requires

CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o.provides: CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/SamplePlugin.dir/build.make CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o.provides.build
.PHONY : CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o.provides

CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o.provides.build: CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o


CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o: CMakeFiles/SamplePlugin.dir/flags.make
CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o: src/moc_SamplePlugin.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o -c /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/src/moc_SamplePlugin.cxx

CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/src/moc_SamplePlugin.cxx > CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.i

CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/src/moc_SamplePlugin.cxx -o CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.s

CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o.requires:

.PHONY : CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o.requires

CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o.provides: CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o.requires
	$(MAKE) -f CMakeFiles/SamplePlugin.dir/build.make CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o.provides.build
.PHONY : CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o.provides

CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o.provides.build: CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o


CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o: CMakeFiles/SamplePlugin.dir/flags.make
CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o: qrc_resources.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o -c /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/qrc_resources.cxx

CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/qrc_resources.cxx > CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.i

CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/qrc_resources.cxx -o CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.s

CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o.requires:

.PHONY : CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o.requires

CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o.provides: CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o.requires
	$(MAKE) -f CMakeFiles/SamplePlugin.dir/build.make CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o.provides.build
.PHONY : CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o.provides

CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o.provides.build: CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o


# Object files for target SamplePlugin
SamplePlugin_OBJECTS = \
"CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o" \
"CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o" \
"CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o"

# External object files for target SamplePlugin
SamplePlugin_EXTERNAL_OBJECTS =

../libs/release/libSamplePlugin.so: CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o
../libs/release/libSamplePlugin.so: CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o
../libs/release/libSamplePlugin.so: CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o
../libs/release/libSamplePlugin.so: CMakeFiles/SamplePlugin.dir/build.make
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_stitching.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_superres.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_videostab.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_aruco.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_bgsegm.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_bioinspired.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_ccalib.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_dnn.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_dpm.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_fuzzy.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_line_descriptor.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_optflow.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_plot.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_reg.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_saliency.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_stereo.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_structured_light.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_surface_matching.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_tracking.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_xfeatures2d.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_ximgproc.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_xobjdetect.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_xphoto.so.3.1.0
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_lua_s.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/liblua51.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_algorithms.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_pathplanners.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_pathoptimization.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_simulation.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_opengl.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_assembly.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_task.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_calibration.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_csg.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_control.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_proximitystrategies.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/libyaobi.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/libpqp.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_assimp.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_qhull.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_csgjs.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_unzip.a
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libz.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_lua_s.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/liblua51.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_algorithms.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_pathplanners.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_pathoptimization.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_simulation.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_opengl.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_assembly.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_task.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_calibration.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_csg.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_control.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_proximitystrategies.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/libyaobi.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/libpqp.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_assimp.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_qhull.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_csgjs.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_unzip.a
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libz.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtOpenGL.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtDesigner.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtUiTools.a
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtGui.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtCore.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libGL.so
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_shape.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_phase_unwrapping.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_rgbd.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_calib3d.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_video.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_datasets.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_face.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_text.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_features2d.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_flann.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_objdetect.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_ml.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_highgui.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_photo.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_videoio.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_imgproc.so.3.1.0
../libs/release/libSamplePlugin.so: /usr/local/lib/libopencv_core.so.3.1.0
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_lua_s.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/liblua51.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_algorithms.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_pathplanners.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_pathoptimization.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_simulation.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_opengl.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_assembly.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_task.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_calibration.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_csg.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_control.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_proximitystrategies.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/libyaobi.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/libpqp.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libxerces-c.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_assimp.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_qhull.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_csgjs.a
../libs/release/libSamplePlugin.so: /home/mat/RobWork/RobWork/libs/release/librw_unzip.a
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libz.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtOpenGL.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtDesigner.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtUiTools.a
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtGui.so
../libs/release/libSamplePlugin.so: /usr/lib/x86_64-linux-gnu/libQtCore.so
../libs/release/libSamplePlugin.so: CMakeFiles/SamplePlugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared module ../libs/release/libSamplePlugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SamplePlugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SamplePlugin.dir/build: ../libs/release/libSamplePlugin.so

.PHONY : CMakeFiles/SamplePlugin.dir/build

CMakeFiles/SamplePlugin.dir/requires: CMakeFiles/SamplePlugin.dir/src/SamplePlugin.cpp.o.requires
CMakeFiles/SamplePlugin.dir/requires: CMakeFiles/SamplePlugin.dir/src/moc_SamplePlugin.cxx.o.requires
CMakeFiles/SamplePlugin.dir/requires: CMakeFiles/SamplePlugin.dir/qrc_resources.cxx.o.requires

.PHONY : CMakeFiles/SamplePlugin.dir/requires

CMakeFiles/SamplePlugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SamplePlugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SamplePlugin.dir/clean

CMakeFiles/SamplePlugin.dir/depend: ui_SamplePlugin.h
CMakeFiles/SamplePlugin.dir/depend: src/moc_SamplePlugin.cxx
CMakeFiles/SamplePlugin.dir/depend: qrc_resources.cxx
	cd /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10 /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10 /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build /home/mat/7_semester_workspace/rovi_final/robotics/SamplePluginPA10/build/CMakeFiles/SamplePlugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SamplePlugin.dir/depend

