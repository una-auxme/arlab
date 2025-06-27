cmake_minimum_required(VERSION 3.8)
project(arlab_knowledge_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

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

# Include any interface dependencies here:
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add all your interface files here:
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EntityType.msg"
  "msg/Result.msg"
  "srv/GetDescription.srv"
  "srv/GetEntities.srv"
  "srv/DoorGetOpen.srv"
  "srv/DoorGetWidth.srv"
  "srv/GetPoseAndReference.srv"
  "srv/GetShape.srv"
  "srv/AddEntity.srv"
  "srv/AddCupboard.srv"
  "srv/AddDoor.srv"
  "srv/AddFurniture.srv"
  "srv/AddHuman.srv"
  "srv/AddPickable.srv"
  "srv/AddShelf.srv"
  "srv/AddTable.srv"
  DEPENDENCIES # Add packages that above messages depend on
  geometry_msgs
)

ament_package()
