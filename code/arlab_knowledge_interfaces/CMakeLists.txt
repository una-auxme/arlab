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
find_package(nav_msgs REQUIRED)

# Add all your interface files here:
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Entity.msg"
  "msg/EntityPickable.msg"
  "msg/EntityHuman.msg"
  "msg/EntityFurniture.msg"
  "msg/EntityFurnitureCupboard.msg"
  "msg/EntityFurnitureDoor.msg"
  "msg/EntityFurnitureShelf.msg"
  "msg/EntityFurnitureTable.msg"

  "msg/EntityType.msg"
  "msg/Result.msg"

  "msg/RobotStatusEvent.msg"
  "msg/MovementStatus.msg"
  "msg/ManipulationStatus.msg"
  "msg/SafetyStatus.msg"

  "srv/GetDescription.srv"
  "srv/GetEntities.srv"
  "srv/GetEntity.srv"
  "srv/GetPose.srv"
  "srv/GetShape.srv"

  "srv/AddEntity.srv"

  "srv/UpdEntity.srv"
  "srv/UpdPose.srv"
  "srv/UpdShape.srv"

  "srv/DelEntities.srv"
  "srv/GetReference.srv"
  "srv/UpdReference.srv"

  "srv/AddMap.srv"
  "srv/GetMap.srv"
  "srv/AddStatusEvent.srv"
  "srv/GetStatusEvents.srv"
  DEPENDENCIES # Add packages that above messages depend on
  geometry_msgs
  nav_msgs
)

ament_package()
