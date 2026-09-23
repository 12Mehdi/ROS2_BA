# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target fairino_msgs::fairino_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${fairino_msgs_TARGETS}.
if(fairino_msgs_TARGETS AND NOT TARGET fairino_msgs::fairino_msgs)
  add_library(fairino_msgs::fairino_msgs INTERFACE IMPORTED)
  set_target_properties(fairino_msgs::fairino_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${fairino_msgs_TARGETS}")
endif()
