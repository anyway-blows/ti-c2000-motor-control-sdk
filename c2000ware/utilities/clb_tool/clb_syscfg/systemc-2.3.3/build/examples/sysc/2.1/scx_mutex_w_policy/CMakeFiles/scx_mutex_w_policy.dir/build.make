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
include examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/depend.make

# Include the progress variables for this target.
include examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/progress.make

# Include the compile flags for this target's objects.
include examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/flags.make

examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.obj: examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/flags.make
examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.obj: examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/includes_CXX.rsp
examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.obj: ../examples/sysc/2.1/scx_mutex_w_policy/scx_mutex_w_policy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.1\scx_mutex_w_policy && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\scx_mutex_w_policy.dir\scx_mutex_w_policy.cpp.obj -c C:\systemc-2.3.3\examples\sysc\2.1\scx_mutex_w_policy\scx_mutex_w_policy.cpp

examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.1\scx_mutex_w_policy && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\2.1\scx_mutex_w_policy\scx_mutex_w_policy.cpp > CMakeFiles\scx_mutex_w_policy.dir\scx_mutex_w_policy.cpp.i

examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.1\scx_mutex_w_policy && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\2.1\scx_mutex_w_policy\scx_mutex_w_policy.cpp -o CMakeFiles\scx_mutex_w_policy.dir\scx_mutex_w_policy.cpp.s

# Object files for target scx_mutex_w_policy
scx_mutex_w_policy_OBJECTS = \
"CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.obj"

# External object files for target scx_mutex_w_policy
scx_mutex_w_policy_EXTERNAL_OBJECTS =

examples/sysc/2.1/scx_mutex_w_policy/scx_mutex_w_policy.exe: examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/scx_mutex_w_policy.cpp.obj
examples/sysc/2.1/scx_mutex_w_policy/scx_mutex_w_policy.exe: examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/build.make
examples/sysc/2.1/scx_mutex_w_policy/scx_mutex_w_policy.exe: src/libsystemc.a
examples/sysc/2.1/scx_mutex_w_policy/scx_mutex_w_policy.exe: examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/linklibs.rsp
examples/sysc/2.1/scx_mutex_w_policy/scx_mutex_w_policy.exe: examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/objects1.rsp
examples/sysc/2.1/scx_mutex_w_policy/scx_mutex_w_policy.exe: examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable scx_mutex_w_policy.exe"
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.1\scx_mutex_w_policy && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\scx_mutex_w_policy.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/build: examples/sysc/2.1/scx_mutex_w_policy/scx_mutex_w_policy.exe

.PHONY : examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/build

examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/clean:
	cd /d C:\systemc-2.3.3\build\examples\sysc\2.1\scx_mutex_w_policy && $(CMAKE_COMMAND) -P CMakeFiles\scx_mutex_w_policy.dir\cmake_clean.cmake
.PHONY : examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/clean

examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\systemc-2.3.3 C:\systemc-2.3.3\examples\sysc\2.1\scx_mutex_w_policy C:\systemc-2.3.3\build C:\systemc-2.3.3\build\examples\sysc\2.1\scx_mutex_w_policy C:\systemc-2.3.3\build\examples\sysc\2.1\scx_mutex_w_policy\CMakeFiles\scx_mutex_w_policy.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : examples/sysc/2.1/scx_mutex_w_policy/CMakeFiles/scx_mutex_w_policy.dir/depend

