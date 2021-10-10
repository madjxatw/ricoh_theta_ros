cmake_minimum_required(VERSION 3.2)

project(equirec2perspec CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

# OpenCV 4 defines some macros in different headers
if(OpenCV_VERSION_MAJOR EQUAL 4)
  add_compile_definitions(OPENCV_VER_4)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS OpenCV
)

include_directories(include "${OpenCV_INCLUDE_DIRS}")

add_library(${PROJECT_NAME} SHARED src/equirec2perspec.cpp)
target_link_libraries(${PROJECT_NAME} "${OpenCV_LIBRARIES}")
target_compile_options(${PROJECT_NAME} PRIVATE -O3)
set_target_properties(${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_NAME equi2pers
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)
