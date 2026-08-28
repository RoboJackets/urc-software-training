cmake_minimum_required(VERSION 3.8)
project(robonav_training_common)

find_package(ament_cmake REQUIRED)

# This package is header-only: there is nothing to compile. We just install the
# headers and tell downstream packages where to find them. A package that does
#   find_package(robonav_training_common REQUIRED)
#   ament_target_dependencies(<target> ... robonav_training_common)
# gets these headers on its include path automatically.
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
