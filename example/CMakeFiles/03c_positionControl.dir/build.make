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
CMAKE_SOURCE_DIR = /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example

# Include any dependencies generated for this target.
include CMakeFiles/03c_positionControl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/03c_positionControl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/03c_positionControl.dir/flags.make

CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o: CMakeFiles/03c_positionControl.dir/flags.make
CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o: src/basic/03c_positionControl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o -c /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/src/basic/03c_positionControl.cpp

CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/src/basic/03c_positionControl.cpp > CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.i

CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/src/basic/03c_positionControl.cpp -o CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.s

CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o.requires:

.PHONY : CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o.requires

CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o.provides: CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o.requires
	$(MAKE) -f CMakeFiles/03c_positionControl.dir/build.make CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o.provides.build
.PHONY : CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o.provides

CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o.provides.build: CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o


# Object files for target 03c_positionControl
03c_positionControl_OBJECTS = \
"CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o"

# External object files for target 03c_positionControl
03c_positionControl_EXTERNAL_OBJECTS =

bin/03c_positionControl: CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o
bin/03c_positionControl: CMakeFiles/03c_positionControl.dir/build.make
bin/03c_positionControl: libDeltaRobot.a
bin/03c_positionControl: CMakeFiles/03c_positionControl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/03c_positionControl"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/03c_positionControl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/03c_positionControl.dir/build: bin/03c_positionControl

.PHONY : CMakeFiles/03c_positionControl.dir/build

CMakeFiles/03c_positionControl.dir/requires: CMakeFiles/03c_positionControl.dir/src/basic/03c_positionControl.cpp.o.requires

.PHONY : CMakeFiles/03c_positionControl.dir/requires

CMakeFiles/03c_positionControl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/03c_positionControl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/03c_positionControl.dir/clean

CMakeFiles/03c_positionControl.dir/depend:
	cd /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/CMakeFiles/03c_positionControl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/03c_positionControl.dir/depend
