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
CMAKE_SOURCE_DIR = "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/build"

# Include any dependencies generated for this target.
include CMakeFiles/DeltaRobotControl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/DeltaRobotControl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DeltaRobotControl.dir/flags.make

CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o: CMakeFiles/DeltaRobotControl.dir/flags.make
CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o: ../src/DeltaRobotControl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o -c "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/src/DeltaRobotControl.cpp"

CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/src/DeltaRobotControl.cpp" > CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.i

CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/src/DeltaRobotControl.cpp" -o CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.s

CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o.requires:

.PHONY : CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o.requires

CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o.provides: CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o.requires
	$(MAKE) -f CMakeFiles/DeltaRobotControl.dir/build.make CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o.provides.build
.PHONY : CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o.provides

CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o.provides.build: CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o


# Object files for target DeltaRobotControl
DeltaRobotControl_OBJECTS = \
"CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o"

# External object files for target DeltaRobotControl
DeltaRobotControl_EXTERNAL_OBJECTS =

lib/libDeltaRobotControl.so: CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o
lib/libDeltaRobotControl.so: CMakeFiles/DeltaRobotControl.dir/build.make
lib/libDeltaRobotControl.so: CMakeFiles/DeltaRobotControl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library lib/libDeltaRobotControl.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DeltaRobotControl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DeltaRobotControl.dir/build: lib/libDeltaRobotControl.so

.PHONY : CMakeFiles/DeltaRobotControl.dir/build

CMakeFiles/DeltaRobotControl.dir/requires: CMakeFiles/DeltaRobotControl.dir/src/DeltaRobotControl.cpp.o.requires

.PHONY : CMakeFiles/DeltaRobotControl.dir/requires

CMakeFiles/DeltaRobotControl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/DeltaRobotControl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/DeltaRobotControl.dir/clean

CMakeFiles/DeltaRobotControl.dir/depend:
	cd "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot" "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot" "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/build" "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/build" "/home/zhonghang/Desktop/delta robot using innfos actuators/deltarobot/build/CMakeFiles/DeltaRobotControl.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/DeltaRobotControl.dir/depend
