# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /snap/clion/61/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/61/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/christ/zyoung/smoking/imagWrap/rectify

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/imo_rectify.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/imo_rectify.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imo_rectify.dir/flags.make

CMakeFiles/imo_rectify.dir/lsd.c.o: CMakeFiles/imo_rectify.dir/flags.make
CMakeFiles/imo_rectify.dir/lsd.c.o: ../lsd.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/imo_rectify.dir/lsd.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/imo_rectify.dir/lsd.c.o   -c /home/christ/zyoung/smoking/imagWrap/rectify/lsd.c

CMakeFiles/imo_rectify.dir/lsd.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/imo_rectify.dir/lsd.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/christ/zyoung/smoking/imagWrap/rectify/lsd.c > CMakeFiles/imo_rectify.dir/lsd.c.i

CMakeFiles/imo_rectify.dir/lsd.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/imo_rectify.dir/lsd.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/christ/zyoung/smoking/imagWrap/rectify/lsd.c -o CMakeFiles/imo_rectify.dir/lsd.c.s

CMakeFiles/imo_rectify.dir/rotationmath.cpp.o: CMakeFiles/imo_rectify.dir/flags.make
CMakeFiles/imo_rectify.dir/rotationmath.cpp.o: ../rotationmath.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/imo_rectify.dir/rotationmath.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imo_rectify.dir/rotationmath.cpp.o -c /home/christ/zyoung/smoking/imagWrap/rectify/rotationmath.cpp

CMakeFiles/imo_rectify.dir/rotationmath.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imo_rectify.dir/rotationmath.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/christ/zyoung/smoking/imagWrap/rectify/rotationmath.cpp > CMakeFiles/imo_rectify.dir/rotationmath.cpp.i

CMakeFiles/imo_rectify.dir/rotationmath.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imo_rectify.dir/rotationmath.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/christ/zyoung/smoking/imagWrap/rectify/rotationmath.cpp -o CMakeFiles/imo_rectify.dir/rotationmath.cpp.s

CMakeFiles/imo_rectify.dir/VPDetection.cpp.o: CMakeFiles/imo_rectify.dir/flags.make
CMakeFiles/imo_rectify.dir/VPDetection.cpp.o: ../VPDetection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/imo_rectify.dir/VPDetection.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imo_rectify.dir/VPDetection.cpp.o -c /home/christ/zyoung/smoking/imagWrap/rectify/VPDetection.cpp

CMakeFiles/imo_rectify.dir/VPDetection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imo_rectify.dir/VPDetection.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/christ/zyoung/smoking/imagWrap/rectify/VPDetection.cpp > CMakeFiles/imo_rectify.dir/VPDetection.cpp.i

CMakeFiles/imo_rectify.dir/VPDetection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imo_rectify.dir/VPDetection.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/christ/zyoung/smoking/imagWrap/rectify/VPDetection.cpp -o CMakeFiles/imo_rectify.dir/VPDetection.cpp.s

CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.o: CMakeFiles/imo_rectify.dir/flags.make
CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.o: ../wrapImageRef.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.o -c /home/christ/zyoung/smoking/imagWrap/rectify/wrapImageRef.cpp

CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/christ/zyoung/smoking/imagWrap/rectify/wrapImageRef.cpp > CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.i

CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/christ/zyoung/smoking/imagWrap/rectify/wrapImageRef.cpp -o CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.s

# Object files for target imo_rectify
imo_rectify_OBJECTS = \
"CMakeFiles/imo_rectify.dir/lsd.c.o" \
"CMakeFiles/imo_rectify.dir/rotationmath.cpp.o" \
"CMakeFiles/imo_rectify.dir/VPDetection.cpp.o" \
"CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.o"

# External object files for target imo_rectify
imo_rectify_EXTERNAL_OBJECTS =

libimo_rectify.so: CMakeFiles/imo_rectify.dir/lsd.c.o
libimo_rectify.so: CMakeFiles/imo_rectify.dir/rotationmath.cpp.o
libimo_rectify.so: CMakeFiles/imo_rectify.dir/VPDetection.cpp.o
libimo_rectify.so: CMakeFiles/imo_rectify.dir/wrapImageRef.cpp.o
libimo_rectify.so: CMakeFiles/imo_rectify.dir/build.make
libimo_rectify.so: CMakeFiles/imo_rectify.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared library libimo_rectify.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imo_rectify.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imo_rectify.dir/build: libimo_rectify.so

.PHONY : CMakeFiles/imo_rectify.dir/build

CMakeFiles/imo_rectify.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imo_rectify.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imo_rectify.dir/clean

CMakeFiles/imo_rectify.dir/depend:
	cd /home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/christ/zyoung/smoking/imagWrap/rectify /home/christ/zyoung/smoking/imagWrap/rectify /home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug /home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug /home/christ/zyoung/smoking/imagWrap/rectify/cmake-build-debug/CMakeFiles/imo_rectify.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imo_rectify.dir/depend

