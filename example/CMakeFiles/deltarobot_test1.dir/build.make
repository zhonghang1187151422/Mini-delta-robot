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
include CMakeFiles/deltarobot_test1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/deltarobot_test1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/deltarobot_test1.dir/flags.make

CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o: CMakeFiles/deltarobot_test1.dir/flags.make
CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o: src/advanced/deltarobot_test1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o -c /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/src/advanced/deltarobot_test1.cpp

CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/src/advanced/deltarobot_test1.cpp > CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.i

CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/src/advanced/deltarobot_test1.cpp -o CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.s

CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o.requires:

.PHONY : CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o.requires

CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o.provides: CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o.requires
	$(MAKE) -f CMakeFiles/deltarobot_test1.dir/build.make CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o.provides.build
.PHONY : CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o.provides

CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o.provides.build: CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o


# Object files for target deltarobot_test1
deltarobot_test1_OBJECTS = \
"CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o"

# External object files for target deltarobot_test1
deltarobot_test1_EXTERNAL_OBJECTS =

bin/deltarobot_test1: CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o
bin/deltarobot_test1: CMakeFiles/deltarobot_test1.dir/build.make
bin/deltarobot_test1: libDeltaRobot.a
bin/deltarobot_test1: CMakeFiles/deltarobot_test1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/deltarobot_test1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/deltarobot_test1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/deltarobot_test1.dir/build: bin/deltarobot_test1

.PHONY : CMakeFiles/deltarobot_test1.dir/build

CMakeFiles/deltarobot_test1.dir/requires: CMakeFiles/deltarobot_test1.dir/src/advanced/deltarobot_test1.cpp.o.requires

.PHONY : CMakeFiles/deltarobot_test1.dir/requires

CMakeFiles/deltarobot_test1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/deltarobot_test1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/deltarobot_test1.dir/clean

CMakeFiles/deltarobot_test1.dir/depend:
	cd /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/CMakeFiles/deltarobot_test1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/deltarobot_test1.dir/depend

