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
CMAKE_SOURCE_DIR = /home/eaibot/Desktop/slambook_jy/BA_PnP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eaibot/Desktop/slambook_jy/BA_PnP/build

# Include any dependencies generated for this target.
include app/CMakeFiles/BA_PnP.dir/depend.make

# Include the progress variables for this target.
include app/CMakeFiles/BA_PnP.dir/progress.make

# Include the compile flags for this target's objects.
include app/CMakeFiles/BA_PnP.dir/flags.make

app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o: app/CMakeFiles/BA_PnP.dir/flags.make
app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o: ../app/BA_PnP.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eaibot/Desktop/slambook_jy/BA_PnP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o"
	cd /home/eaibot/Desktop/slambook_jy/BA_PnP/build/app && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o -c /home/eaibot/Desktop/slambook_jy/BA_PnP/app/BA_PnP.cpp

app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BA_PnP.dir/BA_PnP.cpp.i"
	cd /home/eaibot/Desktop/slambook_jy/BA_PnP/build/app && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eaibot/Desktop/slambook_jy/BA_PnP/app/BA_PnP.cpp > CMakeFiles/BA_PnP.dir/BA_PnP.cpp.i

app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BA_PnP.dir/BA_PnP.cpp.s"
	cd /home/eaibot/Desktop/slambook_jy/BA_PnP/build/app && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eaibot/Desktop/slambook_jy/BA_PnP/app/BA_PnP.cpp -o CMakeFiles/BA_PnP.dir/BA_PnP.cpp.s

app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o.requires:

.PHONY : app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o.requires

app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o.provides: app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o.requires
	$(MAKE) -f app/CMakeFiles/BA_PnP.dir/build.make app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o.provides.build
.PHONY : app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o.provides

app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o.provides.build: app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o


# Object files for target BA_PnP
BA_PnP_OBJECTS = \
"CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o"

# External object files for target BA_PnP
BA_PnP_EXTERNAL_OBJECTS =

../bin/BA_PnP: app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o
../bin/BA_PnP: app/CMakeFiles/BA_PnP.dir/build.make
../bin/BA_PnP: ../lib/libmyslam.so
../bin/BA_PnP: /usr/local/lib/libopencv_shape.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_highgui.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_objdetect.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_dnn.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_viz.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_superres.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_ml.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_videostab.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_stitching.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_photo.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_calib3d.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_videoio.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_imgcodecs.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_video.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_features2d.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_flann.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_imgproc.so.3.4.12
../bin/BA_PnP: /usr/local/lib/libopencv_core.so.3.4.12
../bin/BA_PnP: app/CMakeFiles/BA_PnP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eaibot/Desktop/slambook_jy/BA_PnP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/BA_PnP"
	cd /home/eaibot/Desktop/slambook_jy/BA_PnP/build/app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BA_PnP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app/CMakeFiles/BA_PnP.dir/build: ../bin/BA_PnP

.PHONY : app/CMakeFiles/BA_PnP.dir/build

app/CMakeFiles/BA_PnP.dir/requires: app/CMakeFiles/BA_PnP.dir/BA_PnP.cpp.o.requires

.PHONY : app/CMakeFiles/BA_PnP.dir/requires

app/CMakeFiles/BA_PnP.dir/clean:
	cd /home/eaibot/Desktop/slambook_jy/BA_PnP/build/app && $(CMAKE_COMMAND) -P CMakeFiles/BA_PnP.dir/cmake_clean.cmake
.PHONY : app/CMakeFiles/BA_PnP.dir/clean

app/CMakeFiles/BA_PnP.dir/depend:
	cd /home/eaibot/Desktop/slambook_jy/BA_PnP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eaibot/Desktop/slambook_jy/BA_PnP /home/eaibot/Desktop/slambook_jy/BA_PnP/app /home/eaibot/Desktop/slambook_jy/BA_PnP/build /home/eaibot/Desktop/slambook_jy/BA_PnP/build/app /home/eaibot/Desktop/slambook_jy/BA_PnP/build/app/CMakeFiles/BA_PnP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app/CMakeFiles/BA_PnP.dir/depend

