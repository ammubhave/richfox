# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox/build

# Include any dependencies generated for this target.
include CMakeFiles/richfox.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/richfox.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/richfox.dir/flags.make

CMakeFiles/richfox.dir/src/centroid_listener.o: CMakeFiles/richfox.dir/flags.make
CMakeFiles/richfox.dir/src/centroid_listener.o: ../src/centroid_listener.cpp
CMakeFiles/richfox.dir/src/centroid_listener.o: ../manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/bullet/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/rosconsole/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/rostest/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/roswtf/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/message_filters/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/rosbag/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/roslib/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/bond_core/bond/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/bond_core/smclib/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/bond_core/bondcpp/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/share/rosservice/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet_topic_tools/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/perception_pcl/pcl_ros/manifest.xml
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/bond_core/bond/msg_gen/generated
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/nodelet_core/nodelet/srv_gen/generated
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/msg_gen/generated
CMakeFiles/richfox.dir/src/centroid_listener.o: /opt/ros/fuerte/stacks/dynamic_reconfigure/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/richfox.dir/src/centroid_listener.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/richfox.dir/src/centroid_listener.o -c /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox/src/centroid_listener.cpp

CMakeFiles/richfox.dir/src/centroid_listener.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/richfox.dir/src/centroid_listener.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox/src/centroid_listener.cpp > CMakeFiles/richfox.dir/src/centroid_listener.i

CMakeFiles/richfox.dir/src/centroid_listener.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/richfox.dir/src/centroid_listener.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox/src/centroid_listener.cpp -o CMakeFiles/richfox.dir/src/centroid_listener.s

CMakeFiles/richfox.dir/src/centroid_listener.o.requires:
.PHONY : CMakeFiles/richfox.dir/src/centroid_listener.o.requires

CMakeFiles/richfox.dir/src/centroid_listener.o.provides: CMakeFiles/richfox.dir/src/centroid_listener.o.requires
	$(MAKE) -f CMakeFiles/richfox.dir/build.make CMakeFiles/richfox.dir/src/centroid_listener.o.provides.build
.PHONY : CMakeFiles/richfox.dir/src/centroid_listener.o.provides

CMakeFiles/richfox.dir/src/centroid_listener.o.provides.build: CMakeFiles/richfox.dir/src/centroid_listener.o

# Object files for target richfox
richfox_OBJECTS = \
"CMakeFiles/richfox.dir/src/centroid_listener.o"

# External object files for target richfox
richfox_EXTERNAL_OBJECTS =

../bin/richfox: CMakeFiles/richfox.dir/src/centroid_listener.o
../bin/richfox: CMakeFiles/richfox.dir/build.make
../bin/richfox: CMakeFiles/richfox.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/richfox"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/richfox.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/richfox.dir/build: ../bin/richfox
.PHONY : CMakeFiles/richfox.dir/build

CMakeFiles/richfox.dir/requires: CMakeFiles/richfox.dir/src/centroid_listener.o.requires
.PHONY : CMakeFiles/richfox.dir/requires

CMakeFiles/richfox.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/richfox.dir/cmake_clean.cmake
.PHONY : CMakeFiles/richfox.dir/clean

CMakeFiles/richfox.dir/depend:
	cd /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox/build /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox/build /afs/athena.mit.edu/user/a/m/ambhave/myws/sandbox/richfox/build/CMakeFiles/richfox.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/richfox.dir/depend
