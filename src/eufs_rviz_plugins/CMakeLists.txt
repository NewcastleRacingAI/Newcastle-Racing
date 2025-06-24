cmake_minimum_required(VERSION 3.8)
project(eufs_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(dependencies
        eigen3_cmake_module
        pluginlib
        rviz_common
        rviz_default_plugins
        geometry_msgs
        std_msgs
        visualization_msgs
        eufs_msgs)

set(sources
        src/eufs_rviz_plugins/displays/cone_array_with_covariance/cone_array_with_covariance_display.cpp
        src/eufs_rviz_plugins/displays/waypoint_array_stamped/waypoint_array_stamped_display.cpp)

qt5_wrap_cpp(qt_moc_files "include/eufs_rviz_plugins/displays/cone_array_with_covariance/cone_array_with_covariance_display.hpp") 
qt5_wrap_cpp(qt_moc_files "include/eufs_rviz_plugins/displays/waypoint_array_stamped/waypoint_array_stamped_display.hpp") 

add_library(${PROJECT_NAME} SHARED
        ${qt_moc_files}
        ${sources})

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        ${Qt5Widgets_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen)

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies})

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY icons meshes
        DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common plugins.xml)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
