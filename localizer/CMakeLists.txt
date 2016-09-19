cmake_minimum_required(VERSION 2.8.3)
project(localizer)

## Find catkin macros and libraries.
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages.
find_package(catkin REQUIRED
  cmake_modules
  roscpp
  tf
  tf_conversions
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########
## Specify additional locations of header files.
## Your package locations should be listed before other locations.
include_directories(include)

## Declare a C++ executable
add_executable(localizer3d src/localizer_3d.cpp)
add_executable(localizer4d src/localizer_4d.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(localizer3d
  ${catkin_LIBRARIES}
)
target_link_libraries(localizer4d
  ${catkin_LIBRARIES}
)
