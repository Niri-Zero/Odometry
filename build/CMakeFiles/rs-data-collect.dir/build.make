# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.23.2/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.23.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/kaustubhkhulbe/Documents/realsense

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/kaustubhkhulbe/Documents/realsense/build

# Include any dependencies generated for this target.
include CMakeFiles/rs-data-collect.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/rs-data-collect.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/rs-data-collect.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rs-data-collect.dir/flags.make

CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.o: CMakeFiles/rs-data-collect.dir/flags.make
CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.o: ../rs-data-collect.cpp
CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.o: CMakeFiles/rs-data-collect.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kaustubhkhulbe/Documents/realsense/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.o -MF CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.o.d -o CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.o -c /Users/kaustubhkhulbe/Documents/realsense/rs-data-collect.cpp

CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kaustubhkhulbe/Documents/realsense/rs-data-collect.cpp > CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.i

CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kaustubhkhulbe/Documents/realsense/rs-data-collect.cpp -o CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.s

# Object files for target rs-data-collect
rs__data__collect_OBJECTS = \
"CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.o"

# External object files for target rs-data-collect
rs__data__collect_EXTERNAL_OBJECTS =

rs-data-collect: CMakeFiles/rs-data-collect.dir/rs-data-collect.cpp.o
rs-data-collect: CMakeFiles/rs-data-collect.dir/build.make
rs-data-collect: /opt/homebrew/lib/librealsense2.2.50.0.dylib
rs-data-collect: CMakeFiles/rs-data-collect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/kaustubhkhulbe/Documents/realsense/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rs-data-collect"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rs-data-collect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rs-data-collect.dir/build: rs-data-collect
.PHONY : CMakeFiles/rs-data-collect.dir/build

CMakeFiles/rs-data-collect.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rs-data-collect.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rs-data-collect.dir/clean

CMakeFiles/rs-data-collect.dir/depend:
	cd /Users/kaustubhkhulbe/Documents/realsense/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/kaustubhkhulbe/Documents/realsense /Users/kaustubhkhulbe/Documents/realsense /Users/kaustubhkhulbe/Documents/realsense/build /Users/kaustubhkhulbe/Documents/realsense/build /Users/kaustubhkhulbe/Documents/realsense/build/CMakeFiles/rs-data-collect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rs-data-collect.dir/depend

