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
include examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/depend.make

# Include the progress variables for this target.
include examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/progress.make

# Include the compile flags for this target's objects.
include examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/flags.make

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/main.cpp.obj: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/flags.make
examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/main.cpp.obj: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/includes_CXX.rsp
examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/main.cpp.obj: ../examples/sysc/fft/fft_flpt/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/main.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\fft_flpt.dir\main.cpp.obj -c C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\main.cpp

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fft_flpt.dir/main.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\main.cpp > CMakeFiles\fft_flpt.dir\main.cpp.i

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fft_flpt.dir/main.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\main.cpp -o CMakeFiles\fft_flpt.dir\main.cpp.s

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/source.cpp.obj: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/flags.make
examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/source.cpp.obj: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/includes_CXX.rsp
examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/source.cpp.obj: ../examples/sysc/fft/fft_flpt/source.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/source.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\fft_flpt.dir\source.cpp.obj -c C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\source.cpp

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/source.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fft_flpt.dir/source.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\source.cpp > CMakeFiles\fft_flpt.dir\source.cpp.i

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/source.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fft_flpt.dir/source.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\source.cpp -o CMakeFiles\fft_flpt.dir\source.cpp.s

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/fft.cpp.obj: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/flags.make
examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/fft.cpp.obj: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/includes_CXX.rsp
examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/fft.cpp.obj: ../examples/sysc/fft/fft_flpt/fft.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/fft.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\fft_flpt.dir\fft.cpp.obj -c C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\fft.cpp

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/fft.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fft_flpt.dir/fft.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\fft.cpp > CMakeFiles\fft_flpt.dir\fft.cpp.i

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/fft.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fft_flpt.dir/fft.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\fft.cpp -o CMakeFiles\fft_flpt.dir\fft.cpp.s

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/sink.cpp.obj: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/flags.make
examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/sink.cpp.obj: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/includes_CXX.rsp
examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/sink.cpp.obj: ../examples/sysc/fft/fft_flpt/sink.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/sink.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\fft_flpt.dir\sink.cpp.obj -c C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\sink.cpp

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/sink.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fft_flpt.dir/sink.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\sink.cpp > CMakeFiles\fft_flpt.dir\sink.cpp.i

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/sink.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fft_flpt.dir/sink.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\fft\fft_flpt\sink.cpp -o CMakeFiles\fft_flpt.dir\sink.cpp.s

# Object files for target fft_flpt
fft_flpt_OBJECTS = \
"CMakeFiles/fft_flpt.dir/main.cpp.obj" \
"CMakeFiles/fft_flpt.dir/source.cpp.obj" \
"CMakeFiles/fft_flpt.dir/fft.cpp.obj" \
"CMakeFiles/fft_flpt.dir/sink.cpp.obj"

# External object files for target fft_flpt
fft_flpt_EXTERNAL_OBJECTS =

examples/sysc/fft/fft_flpt/fft_flpt.exe: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/main.cpp.obj
examples/sysc/fft/fft_flpt/fft_flpt.exe: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/source.cpp.obj
examples/sysc/fft/fft_flpt/fft_flpt.exe: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/fft.cpp.obj
examples/sysc/fft/fft_flpt/fft_flpt.exe: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/sink.cpp.obj
examples/sysc/fft/fft_flpt/fft_flpt.exe: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/build.make
examples/sysc/fft/fft_flpt/fft_flpt.exe: src/libsystemc.a
examples/sysc/fft/fft_flpt/fft_flpt.exe: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/linklibs.rsp
examples/sysc/fft/fft_flpt/fft_flpt.exe: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/objects1.rsp
examples/sysc/fft/fft_flpt/fft_flpt.exe: examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable fft_flpt.exe"
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\fft_flpt.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/build: examples/sysc/fft/fft_flpt/fft_flpt.exe

.PHONY : examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/build

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/clean:
	cd /d C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt && $(CMAKE_COMMAND) -P CMakeFiles\fft_flpt.dir\cmake_clean.cmake
.PHONY : examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/clean

examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\systemc-2.3.3 C:\systemc-2.3.3\examples\sysc\fft\fft_flpt C:\systemc-2.3.3\build C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt C:\systemc-2.3.3\build\examples\sysc\fft\fft_flpt\CMakeFiles\fft_flpt.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : examples/sysc/fft/fft_flpt/CMakeFiles/fft_flpt.dir/depend

