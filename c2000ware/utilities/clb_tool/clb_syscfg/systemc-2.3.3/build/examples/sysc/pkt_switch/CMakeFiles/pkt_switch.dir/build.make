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
include examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/depend.make

# Include the progress variables for this target.
include examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/progress.make

# Include the compile flags for this target's objects.
include examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/flags.make

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/main.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/flags.make
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/main.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/includes_CXX.rsp
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/main.cpp.obj: ../examples/sysc/pkt_switch/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/main.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\pkt_switch.dir\main.cpp.obj -c C:\systemc-2.3.3\examples\sysc\pkt_switch\main.cpp

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pkt_switch.dir/main.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\pkt_switch\main.cpp > CMakeFiles\pkt_switch.dir\main.cpp.i

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pkt_switch.dir/main.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\pkt_switch\main.cpp -o CMakeFiles\pkt_switch.dir\main.cpp.s

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/fifo.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/flags.make
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/fifo.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/includes_CXX.rsp
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/fifo.cpp.obj: ../examples/sysc/pkt_switch/fifo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/fifo.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\pkt_switch.dir\fifo.cpp.obj -c C:\systemc-2.3.3\examples\sysc\pkt_switch\fifo.cpp

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/fifo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pkt_switch.dir/fifo.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\pkt_switch\fifo.cpp > CMakeFiles\pkt_switch.dir\fifo.cpp.i

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/fifo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pkt_switch.dir/fifo.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\pkt_switch\fifo.cpp -o CMakeFiles\pkt_switch.dir\fifo.cpp.s

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/sender.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/flags.make
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/sender.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/includes_CXX.rsp
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/sender.cpp.obj: ../examples/sysc/pkt_switch/sender.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/sender.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\pkt_switch.dir\sender.cpp.obj -c C:\systemc-2.3.3\examples\sysc\pkt_switch\sender.cpp

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/sender.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pkt_switch.dir/sender.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\pkt_switch\sender.cpp > CMakeFiles\pkt_switch.dir\sender.cpp.i

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/sender.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pkt_switch.dir/sender.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\pkt_switch\sender.cpp -o CMakeFiles\pkt_switch.dir\sender.cpp.s

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch_clk.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/flags.make
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch_clk.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/includes_CXX.rsp
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch_clk.cpp.obj: ../examples/sysc/pkt_switch/switch_clk.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch_clk.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\pkt_switch.dir\switch_clk.cpp.obj -c C:\systemc-2.3.3\examples\sysc\pkt_switch\switch_clk.cpp

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch_clk.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pkt_switch.dir/switch_clk.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\pkt_switch\switch_clk.cpp > CMakeFiles\pkt_switch.dir\switch_clk.cpp.i

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch_clk.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pkt_switch.dir/switch_clk.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\pkt_switch\switch_clk.cpp -o CMakeFiles\pkt_switch.dir\switch_clk.cpp.s

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/flags.make
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/includes_CXX.rsp
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch.cpp.obj: ../examples/sysc/pkt_switch/switch.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\pkt_switch.dir\switch.cpp.obj -c C:\systemc-2.3.3\examples\sysc\pkt_switch\switch.cpp

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pkt_switch.dir/switch.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\pkt_switch\switch.cpp > CMakeFiles\pkt_switch.dir\switch.cpp.i

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pkt_switch.dir/switch.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\pkt_switch\switch.cpp -o CMakeFiles\pkt_switch.dir\switch.cpp.s

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/receiver.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/flags.make
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/receiver.cpp.obj: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/includes_CXX.rsp
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/receiver.cpp.obj: ../examples/sysc/pkt_switch/receiver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/receiver.cpp.obj"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\pkt_switch.dir\receiver.cpp.obj -c C:\systemc-2.3.3\examples\sysc\pkt_switch\receiver.cpp

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/receiver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pkt_switch.dir/receiver.cpp.i"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\systemc-2.3.3\examples\sysc\pkt_switch\receiver.cpp > CMakeFiles\pkt_switch.dir\receiver.cpp.i

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/receiver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pkt_switch.dir/receiver.cpp.s"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && C:\TDM-GCC-64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\systemc-2.3.3\examples\sysc\pkt_switch\receiver.cpp -o CMakeFiles\pkt_switch.dir\receiver.cpp.s

# Object files for target pkt_switch
pkt_switch_OBJECTS = \
"CMakeFiles/pkt_switch.dir/main.cpp.obj" \
"CMakeFiles/pkt_switch.dir/fifo.cpp.obj" \
"CMakeFiles/pkt_switch.dir/sender.cpp.obj" \
"CMakeFiles/pkt_switch.dir/switch_clk.cpp.obj" \
"CMakeFiles/pkt_switch.dir/switch.cpp.obj" \
"CMakeFiles/pkt_switch.dir/receiver.cpp.obj"

# External object files for target pkt_switch
pkt_switch_EXTERNAL_OBJECTS =

examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/main.cpp.obj
examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/fifo.cpp.obj
examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/sender.cpp.obj
examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch_clk.cpp.obj
examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/switch.cpp.obj
examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/receiver.cpp.obj
examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/build.make
examples/sysc/pkt_switch/pkt_switch.exe: src/libsystemc.a
examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/linklibs.rsp
examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/objects1.rsp
examples/sysc/pkt_switch/pkt_switch.exe: examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\systemc-2.3.3\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable pkt_switch.exe"
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\pkt_switch.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/build: examples/sysc/pkt_switch/pkt_switch.exe

.PHONY : examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/build

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/clean:
	cd /d C:\systemc-2.3.3\build\examples\sysc\pkt_switch && $(CMAKE_COMMAND) -P CMakeFiles\pkt_switch.dir\cmake_clean.cmake
.PHONY : examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/clean

examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\systemc-2.3.3 C:\systemc-2.3.3\examples\sysc\pkt_switch C:\systemc-2.3.3\build C:\systemc-2.3.3\build\examples\sysc\pkt_switch C:\systemc-2.3.3\build\examples\sysc\pkt_switch\CMakeFiles\pkt_switch.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : examples/sysc/pkt_switch/CMakeFiles/pkt_switch.dir/depend

