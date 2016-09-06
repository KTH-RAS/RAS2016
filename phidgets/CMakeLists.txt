cmake_minimum_required(VERSION 2.4.6)
project(phidgets)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs geometry_msgs tf sensor_msgs sound_play move_base_msgs actionlib rviz message_generation roscpp_serialization)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_BUILD_TYPE Debug)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}

)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   accelerometer_params.msg
   encoder_params.msg
   ir_params.msg
   joystick_params.msg
   led_params.msg
   manager_params.msg
   motor_params.msg
   pantilt_params.msg
   phsensor_params.msg
   pid_params.msg
   rfid_params.msg
   servo_params.msg
   spatial_params.msg
   stepper_params.msg
   textlcd_params.msg
   interface_kit_params.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   interface_kit.srv
   lidar_control.srv
   pid.srv
   servo_reference.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
 DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs

)

catkin_package(
  INCLUDE_DIRS 
#  LIBRARIES robot_state_viewer
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs tf sound_play move_base_msgs actionlib rviz message_runtime roscpp_serialization
#  DEPENDS system_lib
)

#common commands for building c++ executables and libraries
#_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#_add_boost_directories()
#_link_boost(${PROJECT_NAME} thread)
#_add_executable(example examples/example.cpp)

#_add_link_flags()
add_executable(advanced_servo src/advanced_servo.cpp)
add_dependencies(advanced_servo phidgets_generate_messages_cpp)

add_executable(advanced_servo_client src/advanced_servo_client.cpp)
add_dependencies(advanced_servo_client phidgets_generate_messages_cpp)

add_executable(high_speed_encoder src/high_speed_encoder.cpp)
add_dependencies(high_speed_encoder phidgets_generate_messages_cpp)

add_executable(high_speed_encoder_client src/high_speed_encoder_client.cpp)
add_dependencies(high_speed_encoder_client phidgets_generate_messages_cpp)

add_executable(motor_control_hc src/motor_control_hc.cpp)
add_dependencies(motor_control_hc phidgets_generate_messages_cpp)

add_executable(motor_control_hc_client src/motor_control_hc_client.cpp)
add_dependencies(motor_control_hc_client phidgets_generate_messages_cpp)

add_executable(interface_kit src/interface_kit.cpp)
add_dependencies(interface_kit phidgets_generate_messages_cpp)

add_executable(interface_kit_client src/interface_kit_client.cpp)
add_dependencies(interface_kit_client phidgets_generate_messages_cpp)

add_executable(accelerometer src/accelerometer.cpp)
add_dependencies(accelerometer phidgets_generate_messages_cpp)

add_executable(accelerometer_client src/accelerometer_client.cpp)
add_dependencies(accelerometer_client phidgets_generate_messages_cpp)

add_executable(manager src/manager.cpp)
add_dependencies(manager phidgets_generate_messages_cpp)

add_executable(manager_client src/manager_client.cpp)
add_dependencies(manager_client phidgets_generate_messages_cpp)

add_executable(stepper src/stepper.cpp)
add_dependencies(stepper phidgets_generate_messages_cpp)

add_executable(stepper_client src/stepper_client.cpp)
add_dependencies(stepper_client phidgets_generate_messages_cpp)

add_executable(odometry src/odometry.cpp)

add_executable(joystick src/joystick.cpp)
add_dependencies(joystick phidgets_generate_messages_cpp)

add_executable(lidar src/lidar.cpp)
add_dependencies(lidar phidgets_generate_messages_cpp)

add_executable(lidar_client src/lidar_client.cpp)
add_dependencies(lidar_client phidgets_generate_messages_cpp)

add_executable(joystick_motor_control src/joystick_motor_control.cpp)


add_executable(joystick_servo_control src/joystick_servo_control.cpp)


add_executable(servo_glimpse src/servo_glimpse.cpp)
add_dependencies(servo_glimpse phidgets_generate_messages_cpp)

add_executable(rfid src/rfid.cpp)
add_dependencies(rfid phidgets_generate_messages_cpp)

add_executable(rfid_client src/rfid_client.cpp)
add_dependencies(rfid_client phidgets_generate_messages_cpp)

add_executable(textlcd src/textlcd.cpp)
add_dependencies(textlcd phidgets_generate_messages_cpp)

add_executable(textlcd_client src/textlcd_client.cpp)
add_dependencies(textlcd_client phidgets_generate_messages_cpp)

add_executable(phsensor src/phsensor.cpp)
add_dependencies(phsensor phidgets_generate_messages_cpp)

add_executable(phsensor_client src/phsensor_client.cpp)
add_dependencies(phsensor_client phidgets_generate_messages_cpp)

add_executable(spatial src/spatial.cpp)
add_dependencies(spatial phidgets_generate_messages_cpp)

add_executable(spatial_client src/spatial_client.cpp)
add_dependencies(spatial_client phidgets_generate_messages_cpp)

add_executable(led src/led.cpp)
add_dependencies(led phidgets_generate_messages_cpp)

add_executable(led_client src/led_client.cpp)
add_dependencies(led_client phidgets_generate_messages_cpp)

add_executable(ir src/ir.cpp)
add_dependencies(ir phidgets_generate_messages_cpp)


add_executable(ir_client src/ir_client.cpp)
add_dependencies(ir_client phidgets_generate_messages_cpp)

target_link_libraries(advanced_servo phidget21 ${catkin_LIBRARIES})
target_link_libraries(advanced_servo_client phidget21 ${catkin_LIBRARIES})
target_link_libraries(high_speed_encoder phidget21 ${catkin_LIBRARIES})
target_link_libraries(high_speed_encoder_client phidget21 ${catkin_LIBRARIES})

target_link_libraries(motor_control_hc phidget21 ${catkin_LIBRARIES})
target_link_libraries(motor_control_hc_client phidget21 ${catkin_LIBRARIES})

target_link_libraries(interface_kit phidget21 ${catkin_LIBRARIES})
target_link_libraries(interface_kit_client phidget21 ${catkin_LIBRARIES})
target_link_libraries(accelerometer phidget21 ${catkin_LIBRARIES})
target_link_libraries(accelerometer_client phidget21 ${catkin_LIBRARIES})
target_link_libraries(manager phidget21 ${catkin_LIBRARIES})
target_link_libraries(manager_client phidget21 ${catkin_LIBRARIES})

target_link_libraries(stepper phidget21 ${catkin_LIBRARIES})
target_link_libraries(stepper_client phidget21 ${catkin_LIBRARIES})

target_link_libraries(odometry phidget21 ${catkin_LIBRARIES})
target_link_libraries(joystick phidget21 ${catkin_LIBRARIES})

target_link_libraries(lidar phidget21 ${catkin_LIBRARIES})
target_link_libraries(lidar_client phidget21 ${catkin_LIBRARIES})

target_link_libraries(joystick_motor_control phidget21 ${catkin_LIBRARIES})
target_link_libraries(joystick_servo_control phidget21 ${catkin_LIBRARIES})
target_link_libraries(servo_glimpse phidget21 ${catkin_LIBRARIES})

target_link_libraries(rfid phidget21 ${catkin_LIBRARIES})
target_link_libraries(rfid_client phidget21 ${catkin_LIBRARIES})

target_link_libraries(textlcd phidget21 ${catkin_LIBRARIES})
target_link_libraries(textlcd_client phidget21 ${catkin_LIBRARIES})

target_link_libraries(phsensor phidget21 ${catkin_LIBRARIES})
target_link_libraries(phsensor_client phidget21 ${catkin_LIBRARIES})


target_link_libraries(spatial phidget21 ${catkin_LIBRARIES})
target_link_libraries(spatial_client phidget21 ${catkin_LIBRARIES})

target_link_libraries(led phidget21 ${catkin_LIBRARIES})
target_link_libraries(led_client phidget21 ${catkin_LIBRARIES})

target_link_libraries(ir phidget21 ${catkin_LIBRARIES})
target_link_libraries(ir_client phidget21 ${catkin_LIBRARIES})
