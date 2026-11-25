cmake_minimum_required(VERSION 3.8)
project(factory_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToStation.action"
  "action/Fr5Task.action"
  "srv/PlaceOrder.srv"
  "srv/GetInventory.srv"
  "srv/RequestInbound.srv"
  "srv/RetryOrCancel.srv"
  "msg/FactoryState.msg"
  "msg/SlotStock.msg"
  "msg/Inventory.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
