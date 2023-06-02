cmake_minimum_required(VERSION 3.8)
project(base_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(base_controller src/base_controller.cpp src/pid.cpp src/basectl.cpp src/serial.cpp)
add_executable(pid_auto_tune src/pid_auto_tune.cpp src/pid.cpp src/basectl.cpp src/serial.cpp)
target_include_directories(base_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/base_controller>
  )
target_compile_features(base_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(pid_auto_tune PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS base_controller
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pid_auto_tune
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
