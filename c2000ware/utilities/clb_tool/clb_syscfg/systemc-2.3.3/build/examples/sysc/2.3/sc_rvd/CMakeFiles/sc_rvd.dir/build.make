# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.13

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\systemc-2.3.3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\systemc-2.3.3\build

# Include any dependencies generated for this target.
include examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/depend.make

# Include the progress variables for this target.
include examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/progress.make

# Include the compile flags for this target's objects.
include examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/flags.make

examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/main.cpp.obj: examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/flags.make
examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/main.cpp.obj: examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/includes_CXX.rsp
examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/main.cpp.obj: ../examples/sysc/2.3/sc_rvd/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/main.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.3\sc_rvd && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\sc_rvd.dir\main.cpp.obj -c C:\systemc-2.3.3\examples\sysc\2.3\sc_rvd\main.cpp

examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sc_rvd.dir/main.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.3\sc_rvd && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\2.3\sc_rvd\main.cpp > CMakeFiles\sc_rvd.dir\main.cpp.i

examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sc_rvd.dir/main.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.3\sc_rvd && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\2.3\sc_rvd\main.cpp -o CMakeFiles\sc_rvd.dir\main.cpp.s

# Object files for target sc_rvd
sc_rvd_OBJECTS = \
"CMakeFiles/sc_rvd.dir/main.cpp.obj"

# External object files for target sc_rvd
sc_rvd_EXTERNAL_OBJECTS =

examples/sysc/2.3/sc_rvd/sc_rvd.exe: examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/main.cpp.obj
examples/sysc/2.3/sc_rvd/sc_rvd.exe: examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/build.make
examples/sysc/2.3/sc_rvd/sc_rvd.exe: src/libsystemc.a
examples/sysc/2.3/sc_rvd/sc_rvd.exe: examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/linklibs.rsp
examples/sysc/2.3/sc_rvd/sc_rvd.exe: examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/objects1.rsp
examples/sysc/2.3/sc_rvd/sc_rvd.exe: examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sc_rvd.exe"
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.3\sc_rvd && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\sc_rvd.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/build: examples/sysc/2.3/sc_rvd/sc_rvd.exe

.PHONY : examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/build

examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/clean:
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.3\sc_rvd && $(CMAKE_COMMAND) -P CMakeFiles\sc_rvd.dir\cmake_clean.cmake
.PHONY : examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/clean

examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\systemc-2.3.3 C:\systemc-2.3.3\examples\sysc\2.3\sc_rvd C:\systemc-2.3.3\build C:\systemc-2.3.3\build\examples\sysc\2.3\sc_rvd C:\systemc-2.3.3\build\examples\sysc\2.3\sc_rvd\CMakeFiles\sc_rvd.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : examples/sysc/2.3/sc_rvd/CMakeFiles/sc_rvd.dir/depend
