# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(test-are-pointing)

# mandatory use of these packages
find_package(YARP 3.5.1 REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(Gazebo REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

add_subdirectory(gazebo)

# mover
add_library(${PROJECT_NAME}-mover SHARED ${CMAKE_SOURCE_DIR}/src/mover.cpp)
target_compile_definitions(${PROJECT_NAME}-mover PRIVATE _USE_MATH_DEFINES)
target_include_directories(${PROJECT_NAME}-mover PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-mover PRIVATE ${GAZEBO_LIBRARIES} YARP::YARP_os)
install(TARGETS ${PROJECT_NAME}-mover LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

# apply patch
#file(GLOB patch_ini ${PROJECT_SOURCE_DIR}/patch/*.ini)
#add_custom_target(apply_patch COMMAND ${CMAKE_COMMAND} -E copy_if_different ${patch_ini}
#                  "/robotology-superbuild-install/share/iCub/conf_manual/iCubGazeboV2_5_visuomanip")
#add_dependencies(${PROJECT_NAME}-mover apply_patch)
