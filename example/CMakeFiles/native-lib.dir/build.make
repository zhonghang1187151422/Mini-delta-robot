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
include CMakeFiles/native-lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/native-lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/native-lib.dir/flags.make

CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o: CMakeFiles/native-lib.dir/flags.make
CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o: /home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o -c /home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp

CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp > CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.i

CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp -o CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.s

CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o.requires:

.PHONY : CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o.requires

CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o.provides: CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o.requires
	$(MAKE) -f CMakeFiles/native-lib.dir/build.make CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o.provides.build
.PHONY : CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o.provides

CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o.provides.build: CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o


# Object files for target native-lib
native__lib_OBJECTS = \
"CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o"

# External object files for target native-lib
native__lib_EXTERNAL_OBJECTS =

libnative-lib.so: CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o
libnative-lib.so: CMakeFiles/native-lib.dir/build.make
libnative-lib.so: CMakeFiles/native-lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libnative-lib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/native-lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/native-lib.dir/build: libnative-lib.so

.PHONY : CMakeFiles/native-lib.dir/build

CMakeFiles/native-lib.dir/requires: CMakeFiles/native-lib.dir/home/zhonghang/innfos_actuator/innfos-cpp-sdk/deltarobot/src/deltarobot.cpp.o.requires

.PHONY : CMakeFiles/native-lib.dir/requires

CMakeFiles/native-lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/native-lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/native-lib.dir/clean

CMakeFiles/native-lib.dir/depend:
	cd /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example /home/zhonghang/innfos_actuator/innfos-cpp-sdk/example/CMakeFiles/native-lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/native-lib.dir/depend

