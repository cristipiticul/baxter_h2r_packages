# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()


if(baxter_kinect_calibration_CONFIG_INCLUDED)
  return()
endif()
set(baxter_kinect_calibration_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(baxter_kinect_calibration_SOURCE_PREFIX /home/brawner/workspace/src/baxter_hackathon/baxter_kinect_calibration)
  set(baxter_kinect_calibration_DEVEL_PREFIX /home/brawner/workspace/src/baxter_hackathon/baxter_kinect_calibration/build/devel)
  set(baxter_kinect_calibration_INSTALL_PREFIX "")
  set(baxter_kinect_calibration_PREFIX ${baxter_kinect_calibration_DEVEL_PREFIX})
else()
  set(baxter_kinect_calibration_SOURCE_PREFIX "")
  set(baxter_kinect_calibration_DEVEL_PREFIX "")
  set(baxter_kinect_calibration_INSTALL_PREFIX /usr/local)
  set(baxter_kinect_calibration_PREFIX ${baxter_kinect_calibration_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'baxter_kinect_calibration' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(baxter_kinect_calibration_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/brawner/workspace/src/baxter_hackathon/baxter_kinect_calibration/build/devel/include" STREQUAL "")
  set(baxter_kinect_calibration_INCLUDE_DIRS "")
  set(_include_dirs "/home/brawner/workspace/src/baxter_hackathon/baxter_kinect_calibration/build/devel/include")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir}" STREQUAL "include")
      get_filename_component(include "${baxter_kinect_calibration_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'baxter_kinect_calibration' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'Michael Ferguson <mike@vanadiumlabs.com>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'baxter_kinect_calibration' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/brawner/workspace/src/baxter_hackathon/baxter_kinect_calibration/${idir}'.  Ask the maintainer 'Michael Ferguson <mike@vanadiumlabs.com>' to fix it.")
    endif()
    _list_append_unique(baxter_kinect_calibration_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^debug|optimized|general$")
    list(APPEND baxter_kinect_calibration_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND baxter_kinect_calibration_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND baxter_kinect_calibration_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/brawner/workspace/src/baxter_hackathon/baxter_kinect_calibration/build/devel/lib;/home/brawner/workspace/devel/lib;/opt/ros/hydro/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(baxter_kinect_calibration_LIBRARY_DIRS ${lib_path})
      list(APPEND baxter_kinect_calibration_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'baxter_kinect_calibration'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND baxter_kinect_calibration_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(baxter_kinect_calibration_EXPORTED_TARGETS "baxter_kinect_calibration_gencfg")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${baxter_kinect_calibration_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "cv_bridge;image_geometry;image_transport;pcl_ros;roscpp;tf;visualization_msgs;std_msgs;tf2;message_runtime;sensor_msgs;geometry_msgs;resource_retriever;pcl_conversions;dynamic_reconfigure")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 baxter_kinect_calibration_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${baxter_kinect_calibration_dep}_FOUND)
      find_package(${baxter_kinect_calibration_dep} REQUIRED)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${baxter_kinect_calibration_dep} REQUIRED ${depend_list})
  endif()
  _list_append_unique(baxter_kinect_calibration_INCLUDE_DIRS ${${baxter_kinect_calibration_dep}_INCLUDE_DIRS})
  _list_append_deduplicate(baxter_kinect_calibration_LIBRARIES ${${baxter_kinect_calibration_dep}_LIBRARIES})
  _list_append_unique(baxter_kinect_calibration_LIBRARY_DIRS ${${baxter_kinect_calibration_dep}_LIBRARY_DIRS})
  list(APPEND baxter_kinect_calibration_EXPORTED_TARGETS ${${baxter_kinect_calibration_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${baxter_kinect_calibration_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
