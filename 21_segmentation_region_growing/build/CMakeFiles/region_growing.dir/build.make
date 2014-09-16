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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/thomio/codes/pcl/21_segmentation_region_growing

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/thomio/codes/pcl/21_segmentation_region_growing/build

# Include any dependencies generated for this target.
include CMakeFiles/region_growing.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/region_growing.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/region_growing.dir/flags.make

CMakeFiles/region_growing.dir/main.cpp.o: CMakeFiles/region_growing.dir/flags.make
CMakeFiles/region_growing.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/thomio/codes/pcl/21_segmentation_region_growing/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/region_growing.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/region_growing.dir/main.cpp.o -c /home/thomio/codes/pcl/21_segmentation_region_growing/main.cpp

CMakeFiles/region_growing.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/region_growing.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/thomio/codes/pcl/21_segmentation_region_growing/main.cpp > CMakeFiles/region_growing.dir/main.cpp.i

CMakeFiles/region_growing.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/region_growing.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/thomio/codes/pcl/21_segmentation_region_growing/main.cpp -o CMakeFiles/region_growing.dir/main.cpp.s

CMakeFiles/region_growing.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/region_growing.dir/main.cpp.o.requires

CMakeFiles/region_growing.dir/main.cpp.o.provides: CMakeFiles/region_growing.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/region_growing.dir/build.make CMakeFiles/region_growing.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/region_growing.dir/main.cpp.o.provides

CMakeFiles/region_growing.dir/main.cpp.o.provides.build: CMakeFiles/region_growing.dir/main.cpp.o

# Object files for target region_growing
region_growing_OBJECTS = \
"CMakeFiles/region_growing.dir/main.cpp.o"

# External object files for target region_growing
region_growing_EXTERNAL_OBJECTS =

region_growing: CMakeFiles/region_growing.dir/main.cpp.o
region_growing: CMakeFiles/region_growing.dir/build.make
region_growing: /usr/lib/x86_64-linux-gnu/libboost_system.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_thread.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
region_growing: /usr/lib/x86_64-linux-gnu/libpthread.so
region_growing: /usr/lib/libpcl_common.so
region_growing: /usr/lib/libpcl_octree.so
region_growing: /usr/lib/libOpenNI.so
region_growing: /usr/lib/libvtkCommon.so.5.8.0
region_growing: /usr/lib/libvtkRendering.so.5.8.0
region_growing: /usr/lib/libvtkHybrid.so.5.8.0
region_growing: /usr/lib/libvtkCharts.so.5.8.0
region_growing: /usr/lib/libpcl_io.so
region_growing: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
region_growing: /usr/lib/libpcl_kdtree.so
region_growing: /usr/lib/libpcl_search.so
region_growing: /usr/lib/libpcl_sample_consensus.so
region_growing: /usr/lib/libpcl_filters.so
region_growing: /usr/lib/libpcl_features.so
region_growing: /usr/lib/libpcl_registration.so
region_growing: /usr/lib/libpcl_recognition.so
region_growing: /usr/lib/libpcl_visualization.so
region_growing: /usr/lib/libpcl_segmentation.so
region_growing: /usr/lib/libqhull.so
region_growing: /usr/lib/libpcl_surface.so
region_growing: /usr/lib/libpcl_keypoints.so
region_growing: /usr/lib/libpcl_tracking.so
region_growing: /usr/lib/libpcl_apps.so
region_growing: /usr/lib/libpcl_people.so
region_growing: /usr/lib/libpcl_outofcore.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_system.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_thread.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
region_growing: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
region_growing: /usr/lib/x86_64-linux-gnu/libpthread.so
region_growing: /usr/lib/libqhull.so
region_growing: /usr/lib/libOpenNI.so
region_growing: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
region_growing: /usr/lib/libvtkCommon.so.5.8.0
region_growing: /usr/lib/libvtkRendering.so.5.8.0
region_growing: /usr/lib/libvtkHybrid.so.5.8.0
region_growing: /usr/lib/libvtkCharts.so.5.8.0
region_growing: /usr/lib/libpcl_common.so
region_growing: /usr/lib/libpcl_octree.so
region_growing: /usr/lib/libpcl_io.so
region_growing: /usr/lib/libpcl_kdtree.so
region_growing: /usr/lib/libpcl_search.so
region_growing: /usr/lib/libpcl_sample_consensus.so
region_growing: /usr/lib/libpcl_filters.so
region_growing: /usr/lib/libpcl_features.so
region_growing: /usr/lib/libpcl_registration.so
region_growing: /usr/lib/libpcl_recognition.so
region_growing: /usr/lib/libpcl_visualization.so
region_growing: /usr/lib/libpcl_segmentation.so
region_growing: /usr/lib/libpcl_surface.so
region_growing: /usr/lib/libpcl_keypoints.so
region_growing: /usr/lib/libpcl_tracking.so
region_growing: /usr/lib/libpcl_apps.so
region_growing: /usr/lib/libpcl_people.so
region_growing: /usr/lib/libpcl_outofcore.so
region_growing: /usr/lib/libvtkViews.so.5.8.0
region_growing: /usr/lib/libvtkInfovis.so.5.8.0
region_growing: /usr/lib/libvtkWidgets.so.5.8.0
region_growing: /usr/lib/libvtkHybrid.so.5.8.0
region_growing: /usr/lib/libvtkParallel.so.5.8.0
region_growing: /usr/lib/libvtkVolumeRendering.so.5.8.0
region_growing: /usr/lib/libvtkRendering.so.5.8.0
region_growing: /usr/lib/libvtkGraphics.so.5.8.0
region_growing: /usr/lib/libvtkImaging.so.5.8.0
region_growing: /usr/lib/libvtkIO.so.5.8.0
region_growing: /usr/lib/libvtkFiltering.so.5.8.0
region_growing: /usr/lib/libvtkCommon.so.5.8.0
region_growing: /usr/lib/libvtksys.so.5.8.0
region_growing: CMakeFiles/region_growing.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable region_growing"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/region_growing.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/region_growing.dir/build: region_growing
.PHONY : CMakeFiles/region_growing.dir/build

CMakeFiles/region_growing.dir/requires: CMakeFiles/region_growing.dir/main.cpp.o.requires
.PHONY : CMakeFiles/region_growing.dir/requires

CMakeFiles/region_growing.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/region_growing.dir/cmake_clean.cmake
.PHONY : CMakeFiles/region_growing.dir/clean

CMakeFiles/region_growing.dir/depend:
	cd /home/thomio/codes/pcl/21_segmentation_region_growing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thomio/codes/pcl/21_segmentation_region_growing /home/thomio/codes/pcl/21_segmentation_region_growing /home/thomio/codes/pcl/21_segmentation_region_growing/build /home/thomio/codes/pcl/21_segmentation_region_growing/build /home/thomio/codes/pcl/21_segmentation_region_growing/build/CMakeFiles/region_growing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/region_growing.dir/depend
