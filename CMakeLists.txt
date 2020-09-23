cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(Openmp)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
#  CATKIN_DEPENDS eigen_conversion geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "third_party/eigen3"
  ${YAML_CPP_INCLUDEDIR}
)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/)


#FILE(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

#list(APPEND SOURCE_FILES
#        ${PROJECT_SOURCE_DIR}/localization/src
#        )

list(APPEND LIBS
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${catkin_LIBRARIES}
        libGeographiccc
        ${PROJECT_SOURCE_DIR}/lib/libndt_omp.so
        )

#add_executable(match_map src/match_map.cpp ${SOURCE_FILES})
#target_link_libraries(match_map ${LIBS})

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES}  ${LIBS})

#add_executable(back_end_node src/apps/back_end_node.cpp ${ALL_SRCS})
#add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS} optimizeMap_gencpp)
#target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} )


#add_executable(viewer_node src/apps/viewer_node.cpp ${ALL_SRCS})
#add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
#target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} )

add_executable(matching_node src/apps/matching_node.cpp ${ALL_SRCS})
add_dependencies(matching_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(matching_node ${catkin_LIBRARIES}  ${LIBS})