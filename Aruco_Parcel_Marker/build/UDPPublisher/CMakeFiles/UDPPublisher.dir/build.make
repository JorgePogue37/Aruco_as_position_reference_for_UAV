# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/grvc/Demo_Navantia/Aruco_Parcel_Marker

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build

# Include any dependencies generated for this target.
include UDPPublisher/CMakeFiles/UDPPublisher.dir/depend.make

# Include the progress variables for this target.
include UDPPublisher/CMakeFiles/UDPPublisher.dir/progress.make

# Include the compile flags for this target's objects.
include UDPPublisher/CMakeFiles/UDPPublisher.dir/flags.make

UDPPublisher/CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.o: UDPPublisher/CMakeFiles/UDPPublisher.dir/flags.make
UDPPublisher/CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.o: ../UDPPublisher/UDPPublisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object UDPPublisher/CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.o"
	cd /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/UDPPublisher && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.o -c /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/UDPPublisher/UDPPublisher.cpp

UDPPublisher/CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.i"
	cd /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/UDPPublisher && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/UDPPublisher/UDPPublisher.cpp > CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.i

UDPPublisher/CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.s"
	cd /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/UDPPublisher && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/UDPPublisher/UDPPublisher.cpp -o CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.s

# Object files for target UDPPublisher
UDPPublisher_OBJECTS = \
"CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.o"

# External object files for target UDPPublisher
UDPPublisher_EXTERNAL_OBJECTS =

UDPPublisher/libUDPPublisher.a: UDPPublisher/CMakeFiles/UDPPublisher.dir/UDPPublisher.cpp.o
UDPPublisher/libUDPPublisher.a: UDPPublisher/CMakeFiles/UDPPublisher.dir/build.make
UDPPublisher/libUDPPublisher.a: UDPPublisher/CMakeFiles/UDPPublisher.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libUDPPublisher.a"
	cd /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/UDPPublisher && $(CMAKE_COMMAND) -P CMakeFiles/UDPPublisher.dir/cmake_clean_target.cmake
	cd /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/UDPPublisher && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/UDPPublisher.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
UDPPublisher/CMakeFiles/UDPPublisher.dir/build: UDPPublisher/libUDPPublisher.a

.PHONY : UDPPublisher/CMakeFiles/UDPPublisher.dir/build

UDPPublisher/CMakeFiles/UDPPublisher.dir/clean:
	cd /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/UDPPublisher && $(CMAKE_COMMAND) -P CMakeFiles/UDPPublisher.dir/cmake_clean.cmake
.PHONY : UDPPublisher/CMakeFiles/UDPPublisher.dir/clean

UDPPublisher/CMakeFiles/UDPPublisher.dir/depend:
	cd /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/grvc/Demo_Navantia/Aruco_Parcel_Marker /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/UDPPublisher /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/UDPPublisher /home/grvc/Demo_Navantia/Aruco_Parcel_Marker/build/UDPPublisher/CMakeFiles/UDPPublisher.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : UDPPublisher/CMakeFiles/UDPPublisher.dir/depend

