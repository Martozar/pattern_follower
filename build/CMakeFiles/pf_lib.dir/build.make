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
CMAKE_SOURCE_DIR = /home/michail/pattern_follower

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/michail/pattern_follower/build

# Include any dependencies generated for this target.
include CMakeFiles/pf_lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pf_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pf_lib.dir/flags.make

CMakeFiles/pf_lib.dir/src/roidetector.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/roidetector.cpp.o: ../src/roidetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pf_lib.dir/src/roidetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/roidetector.cpp.o -c /home/michail/pattern_follower/src/roidetector.cpp

CMakeFiles/pf_lib.dir/src/roidetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/roidetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/roidetector.cpp > CMakeFiles/pf_lib.dir/src/roidetector.cpp.i

CMakeFiles/pf_lib.dir/src/roidetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/roidetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/roidetector.cpp -o CMakeFiles/pf_lib.dir/src/roidetector.cpp.s

CMakeFiles/pf_lib.dir/src/roidetector.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/roidetector.cpp.o.requires

CMakeFiles/pf_lib.dir/src/roidetector.cpp.o.provides: CMakeFiles/pf_lib.dir/src/roidetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/roidetector.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/roidetector.cpp.o.provides

CMakeFiles/pf_lib.dir/src/roidetector.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/roidetector.cpp.o


CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o: ../src/tamplatematcher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o -c /home/michail/pattern_follower/src/tamplatematcher.cpp

CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/tamplatematcher.cpp > CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.i

CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/tamplatematcher.cpp -o CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.s

CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o.requires

CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o.provides: CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o.provides

CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o


CMakeFiles/pf_lib.dir/src/er1robot.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/er1robot.cpp.o: ../src/er1robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/pf_lib.dir/src/er1robot.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/er1robot.cpp.o -c /home/michail/pattern_follower/src/er1robot.cpp

CMakeFiles/pf_lib.dir/src/er1robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/er1robot.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/er1robot.cpp > CMakeFiles/pf_lib.dir/src/er1robot.cpp.i

CMakeFiles/pf_lib.dir/src/er1robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/er1robot.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/er1robot.cpp -o CMakeFiles/pf_lib.dir/src/er1robot.cpp.s

CMakeFiles/pf_lib.dir/src/er1robot.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/er1robot.cpp.o.requires

CMakeFiles/pf_lib.dir/src/er1robot.cpp.o.provides: CMakeFiles/pf_lib.dir/src/er1robot.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/er1robot.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/er1robot.cpp.o.provides

CMakeFiles/pf_lib.dir/src/er1robot.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/er1robot.cpp.o


CMakeFiles/pf_lib.dir/src/pattern.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/pattern.cpp.o: ../src/pattern.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/pf_lib.dir/src/pattern.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/pattern.cpp.o -c /home/michail/pattern_follower/src/pattern.cpp

CMakeFiles/pf_lib.dir/src/pattern.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/pattern.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/pattern.cpp > CMakeFiles/pf_lib.dir/src/pattern.cpp.i

CMakeFiles/pf_lib.dir/src/pattern.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/pattern.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/pattern.cpp -o CMakeFiles/pf_lib.dir/src/pattern.cpp.s

CMakeFiles/pf_lib.dir/src/pattern.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/pattern.cpp.o.requires

CMakeFiles/pf_lib.dir/src/pattern.cpp.o.provides: CMakeFiles/pf_lib.dir/src/pattern.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/pattern.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/pattern.cpp.o.provides

CMakeFiles/pf_lib.dir/src/pattern.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/pattern.cpp.o


CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o: ../src/contourfinder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o -c /home/michail/pattern_follower/src/contourfinder.cpp

CMakeFiles/pf_lib.dir/src/contourfinder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/contourfinder.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/contourfinder.cpp > CMakeFiles/pf_lib.dir/src/contourfinder.cpp.i

CMakeFiles/pf_lib.dir/src/contourfinder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/contourfinder.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/contourfinder.cpp -o CMakeFiles/pf_lib.dir/src/contourfinder.cpp.s

CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o.requires

CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o.provides: CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o.provides

CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o


CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o: ../src/arucodetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o -c /home/michail/pattern_follower/src/arucodetector.cpp

CMakeFiles/pf_lib.dir/src/arucodetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/arucodetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/arucodetector.cpp > CMakeFiles/pf_lib.dir/src/arucodetector.cpp.i

CMakeFiles/pf_lib.dir/src/arucodetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/arucodetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/arucodetector.cpp -o CMakeFiles/pf_lib.dir/src/arucodetector.cpp.s

CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o.requires

CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o.provides: CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o.provides

CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o


CMakeFiles/pf_lib.dir/src/robot.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/robot.cpp.o: ../src/robot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/pf_lib.dir/src/robot.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/robot.cpp.o -c /home/michail/pattern_follower/src/robot.cpp

CMakeFiles/pf_lib.dir/src/robot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/robot.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/robot.cpp > CMakeFiles/pf_lib.dir/src/robot.cpp.i

CMakeFiles/pf_lib.dir/src/robot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/robot.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/robot.cpp -o CMakeFiles/pf_lib.dir/src/robot.cpp.s

CMakeFiles/pf_lib.dir/src/robot.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/robot.cpp.o.requires

CMakeFiles/pf_lib.dir/src/robot.cpp.o.provides: CMakeFiles/pf_lib.dir/src/robot.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/robot.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/robot.cpp.o.provides

CMakeFiles/pf_lib.dir/src/robot.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/robot.cpp.o


CMakeFiles/pf_lib.dir/src/parser.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/parser.cpp.o: ../src/parser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/pf_lib.dir/src/parser.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/parser.cpp.o -c /home/michail/pattern_follower/src/parser.cpp

CMakeFiles/pf_lib.dir/src/parser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/parser.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/parser.cpp > CMakeFiles/pf_lib.dir/src/parser.cpp.i

CMakeFiles/pf_lib.dir/src/parser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/parser.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/parser.cpp -o CMakeFiles/pf_lib.dir/src/parser.cpp.s

CMakeFiles/pf_lib.dir/src/parser.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/parser.cpp.o.requires

CMakeFiles/pf_lib.dir/src/parser.cpp.o.provides: CMakeFiles/pf_lib.dir/src/parser.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/parser.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/parser.cpp.o.provides

CMakeFiles/pf_lib.dir/src/parser.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/parser.cpp.o


CMakeFiles/pf_lib.dir/src/detector.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/detector.cpp.o: ../src/detector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/pf_lib.dir/src/detector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/detector.cpp.o -c /home/michail/pattern_follower/src/detector.cpp

CMakeFiles/pf_lib.dir/src/detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/detector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/detector.cpp > CMakeFiles/pf_lib.dir/src/detector.cpp.i

CMakeFiles/pf_lib.dir/src/detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/detector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/detector.cpp -o CMakeFiles/pf_lib.dir/src/detector.cpp.s

CMakeFiles/pf_lib.dir/src/detector.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/detector.cpp.o.requires

CMakeFiles/pf_lib.dir/src/detector.cpp.o.provides: CMakeFiles/pf_lib.dir/src/detector.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/detector.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/detector.cpp.o.provides

CMakeFiles/pf_lib.dir/src/detector.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/detector.cpp.o


CMakeFiles/pf_lib.dir/src/measurement.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/measurement.cpp.o: ../src/measurement.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/pf_lib.dir/src/measurement.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/measurement.cpp.o -c /home/michail/pattern_follower/src/measurement.cpp

CMakeFiles/pf_lib.dir/src/measurement.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/measurement.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/measurement.cpp > CMakeFiles/pf_lib.dir/src/measurement.cpp.i

CMakeFiles/pf_lib.dir/src/measurement.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/measurement.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/measurement.cpp -o CMakeFiles/pf_lib.dir/src/measurement.cpp.s

CMakeFiles/pf_lib.dir/src/measurement.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/measurement.cpp.o.requires

CMakeFiles/pf_lib.dir/src/measurement.cpp.o.provides: CMakeFiles/pf_lib.dir/src/measurement.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/measurement.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/measurement.cpp.o.provides

CMakeFiles/pf_lib.dir/src/measurement.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/measurement.cpp.o


CMakeFiles/pf_lib.dir/src/pid.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/pid.cpp.o: ../src/pid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/pf_lib.dir/src/pid.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/pid.cpp.o -c /home/michail/pattern_follower/src/pid.cpp

CMakeFiles/pf_lib.dir/src/pid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/pid.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/pid.cpp > CMakeFiles/pf_lib.dir/src/pid.cpp.i

CMakeFiles/pf_lib.dir/src/pid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/pid.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/pid.cpp -o CMakeFiles/pf_lib.dir/src/pid.cpp.s

CMakeFiles/pf_lib.dir/src/pid.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/pid.cpp.o.requires

CMakeFiles/pf_lib.dir/src/pid.cpp.o.provides: CMakeFiles/pf_lib.dir/src/pid.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/pid.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/pid.cpp.o.provides

CMakeFiles/pf_lib.dir/src/pid.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/pid.cpp.o


CMakeFiles/pf_lib.dir/src/rcm.cpp.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/rcm.cpp.o: ../src/rcm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/pf_lib.dir/src/rcm.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/rcm.cpp.o -c /home/michail/pattern_follower/src/rcm.cpp

CMakeFiles/pf_lib.dir/src/rcm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/rcm.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/rcm.cpp > CMakeFiles/pf_lib.dir/src/rcm.cpp.i

CMakeFiles/pf_lib.dir/src/rcm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/rcm.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/rcm.cpp -o CMakeFiles/pf_lib.dir/src/rcm.cpp.s

CMakeFiles/pf_lib.dir/src/rcm.cpp.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/rcm.cpp.o.requires

CMakeFiles/pf_lib.dir/src/rcm.cpp.o.provides: CMakeFiles/pf_lib.dir/src/rcm.cpp.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/rcm.cpp.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/rcm.cpp.o.provides

CMakeFiles/pf_lib.dir/src/rcm.cpp.o.provides.build: CMakeFiles/pf_lib.dir/src/rcm.cpp.o


CMakeFiles/pf_lib.dir/src/thread_util.cc.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/thread_util.cc.o: ../src/thread_util.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/pf_lib.dir/src/thread_util.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/thread_util.cc.o -c /home/michail/pattern_follower/src/thread_util.cc

CMakeFiles/pf_lib.dir/src/thread_util.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/thread_util.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/thread_util.cc > CMakeFiles/pf_lib.dir/src/thread_util.cc.i

CMakeFiles/pf_lib.dir/src/thread_util.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/thread_util.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/thread_util.cc -o CMakeFiles/pf_lib.dir/src/thread_util.cc.s

CMakeFiles/pf_lib.dir/src/thread_util.cc.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/thread_util.cc.o.requires

CMakeFiles/pf_lib.dir/src/thread_util.cc.o.provides: CMakeFiles/pf_lib.dir/src/thread_util.cc.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/thread_util.cc.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/thread_util.cc.o.provides

CMakeFiles/pf_lib.dir/src/thread_util.cc.o.provides.build: CMakeFiles/pf_lib.dir/src/thread_util.cc.o


CMakeFiles/pf_lib.dir/src/pos_client.cc.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/pos_client.cc.o: ../src/pos_client.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/pf_lib.dir/src/pos_client.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/pos_client.cc.o -c /home/michail/pattern_follower/src/pos_client.cc

CMakeFiles/pf_lib.dir/src/pos_client.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/pos_client.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/pos_client.cc > CMakeFiles/pf_lib.dir/src/pos_client.cc.i

CMakeFiles/pf_lib.dir/src/pos_client.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/pos_client.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/pos_client.cc -o CMakeFiles/pf_lib.dir/src/pos_client.cc.s

CMakeFiles/pf_lib.dir/src/pos_client.cc.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/pos_client.cc.o.requires

CMakeFiles/pf_lib.dir/src/pos_client.cc.o.provides: CMakeFiles/pf_lib.dir/src/pos_client.cc.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/pos_client.cc.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/pos_client.cc.o.provides

CMakeFiles/pf_lib.dir/src/pos_client.cc.o.provides.build: CMakeFiles/pf_lib.dir/src/pos_client.cc.o


CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o: ../src/cam_calibration.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o -c /home/michail/pattern_follower/src/cam_calibration.cc

CMakeFiles/pf_lib.dir/src/cam_calibration.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/cam_calibration.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/cam_calibration.cc > CMakeFiles/pf_lib.dir/src/cam_calibration.cc.i

CMakeFiles/pf_lib.dir/src/cam_calibration.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/cam_calibration.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/cam_calibration.cc -o CMakeFiles/pf_lib.dir/src/cam_calibration.cc.s

CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o.requires

CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o.provides: CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o.provides

CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o.provides.build: CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o


CMakeFiles/pf_lib.dir/src/text_socket.cc.o: CMakeFiles/pf_lib.dir/flags.make
CMakeFiles/pf_lib.dir/src/text_socket.cc.o: ../src/text_socket.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/pf_lib.dir/src/text_socket.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pf_lib.dir/src/text_socket.cc.o -c /home/michail/pattern_follower/src/text_socket.cc

CMakeFiles/pf_lib.dir/src/text_socket.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pf_lib.dir/src/text_socket.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michail/pattern_follower/src/text_socket.cc > CMakeFiles/pf_lib.dir/src/text_socket.cc.i

CMakeFiles/pf_lib.dir/src/text_socket.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pf_lib.dir/src/text_socket.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michail/pattern_follower/src/text_socket.cc -o CMakeFiles/pf_lib.dir/src/text_socket.cc.s

CMakeFiles/pf_lib.dir/src/text_socket.cc.o.requires:

.PHONY : CMakeFiles/pf_lib.dir/src/text_socket.cc.o.requires

CMakeFiles/pf_lib.dir/src/text_socket.cc.o.provides: CMakeFiles/pf_lib.dir/src/text_socket.cc.o.requires
	$(MAKE) -f CMakeFiles/pf_lib.dir/build.make CMakeFiles/pf_lib.dir/src/text_socket.cc.o.provides.build
.PHONY : CMakeFiles/pf_lib.dir/src/text_socket.cc.o.provides

CMakeFiles/pf_lib.dir/src/text_socket.cc.o.provides.build: CMakeFiles/pf_lib.dir/src/text_socket.cc.o


# Object files for target pf_lib
pf_lib_OBJECTS = \
"CMakeFiles/pf_lib.dir/src/roidetector.cpp.o" \
"CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o" \
"CMakeFiles/pf_lib.dir/src/er1robot.cpp.o" \
"CMakeFiles/pf_lib.dir/src/pattern.cpp.o" \
"CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o" \
"CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o" \
"CMakeFiles/pf_lib.dir/src/robot.cpp.o" \
"CMakeFiles/pf_lib.dir/src/parser.cpp.o" \
"CMakeFiles/pf_lib.dir/src/detector.cpp.o" \
"CMakeFiles/pf_lib.dir/src/measurement.cpp.o" \
"CMakeFiles/pf_lib.dir/src/pid.cpp.o" \
"CMakeFiles/pf_lib.dir/src/rcm.cpp.o" \
"CMakeFiles/pf_lib.dir/src/thread_util.cc.o" \
"CMakeFiles/pf_lib.dir/src/pos_client.cc.o" \
"CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o" \
"CMakeFiles/pf_lib.dir/src/text_socket.cc.o"

# External object files for target pf_lib
pf_lib_EXTERNAL_OBJECTS =

libpf_lib.so: CMakeFiles/pf_lib.dir/src/roidetector.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/er1robot.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/pattern.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/robot.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/parser.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/detector.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/measurement.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/pid.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/rcm.cpp.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/thread_util.cc.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/pos_client.cc.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o
libpf_lib.so: CMakeFiles/pf_lib.dir/src/text_socket.cc.o
libpf_lib.so: CMakeFiles/pf_lib.dir/build.make
libpf_lib.so: /usr/local/lib/libopencv_stitching.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_superres.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_videostab.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_aruco.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_bgsegm.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_bioinspired.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_ccalib.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_dnn.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_dpm.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_fuzzy.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_line_descriptor.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_optflow.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_plot.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_reg.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_saliency.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_stereo.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_structured_light.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_surface_matching.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_tracking.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_xfeatures2d.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_ximgproc.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_xobjdetect.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_xphoto.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_shape.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_rgbd.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_calib3d.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_video.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_datasets.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_face.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_text.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_features2d.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_flann.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_objdetect.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_ml.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_highgui.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_photo.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_videoio.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_imgproc.so.3.1.0
libpf_lib.so: /usr/local/lib/libopencv_core.so.3.1.0
libpf_lib.so: CMakeFiles/pf_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/michail/pattern_follower/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Linking CXX shared library libpf_lib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pf_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pf_lib.dir/build: libpf_lib.so

.PHONY : CMakeFiles/pf_lib.dir/build

CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/roidetector.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/tamplatematcher.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/er1robot.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/pattern.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/contourfinder.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/arucodetector.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/robot.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/parser.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/detector.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/measurement.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/pid.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/rcm.cpp.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/thread_util.cc.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/pos_client.cc.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/cam_calibration.cc.o.requires
CMakeFiles/pf_lib.dir/requires: CMakeFiles/pf_lib.dir/src/text_socket.cc.o.requires

.PHONY : CMakeFiles/pf_lib.dir/requires

CMakeFiles/pf_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pf_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pf_lib.dir/clean

CMakeFiles/pf_lib.dir/depend:
	cd /home/michail/pattern_follower/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/michail/pattern_follower /home/michail/pattern_follower /home/michail/pattern_follower/build /home/michail/pattern_follower/build /home/michail/pattern_follower/build/CMakeFiles/pf_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pf_lib.dir/depend

