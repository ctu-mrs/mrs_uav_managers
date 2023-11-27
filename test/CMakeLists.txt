find_package(rostest REQUIRED)

# takeoff test

add_rostest_gtest(MrsUavManagers_UavManagerTakeoffTest
  uav_manager/takeoff/uav_manager_takeoff.test
  uav_manager/takeoff/takeoff_test.cpp
  )

target_link_libraries(
  MrsUavManagers_UavManagerTakeoffTest
  ${catkin_LIBRARIES}
  )

add_dependencies(
  MrsUavManagers_UavManagerTakeoffTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# landing test

add_rostest_gtest(MrsUavManagers_UavManagerLandingTest
  uav_manager/landing/uav_manager_landing.test
  uav_manager/landing/landing_test.cpp
  )

target_link_libraries(
  MrsUavManagers_UavManagerLandingTest
  ${catkin_LIBRARIES}
  )

add_dependencies(
  MrsUavManagers_UavManagerLandingTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# goto service test

add_rostest_gtest(MrsUavManagers_ControlManagerGotoServiceTest
  control_manager/goto_service/control_manager_goto_service.test
  control_manager/goto_service/goto_service_test.cpp
  )

target_link_libraries(
  MrsUavManagers_ControlManagerGotoServiceTest
  ${catkin_LIBRARIES}
  )

add_dependencies(
  MrsUavManagers_ControlManagerGotoServiceTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# eland innovation test

add_rostest_gtest(MrsUavManagers_ControlManagerElandInnovationTest
  control_manager/eland_innovation/control_manager_eland_innovation.test
  control_manager/eland_innovation/eland_innovation_test.cpp
  )

target_link_libraries(
  MrsUavManagers_ControlManagerElandInnovationTest
  ${catkin_LIBRARIES}
  )

add_dependencies(
  MrsUavManagers_ControlManagerElandInnovationTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# eland controlerror test

add_rostest_gtest(MrsUavManagers_ControlManagerElandControlErrorTest
  control_manager/eland_control_error/control_manager_eland_control_error.test
  control_manager/eland_control_error/eland_control_error_test.cpp
  )

target_link_libraries(
  MrsUavManagers_ControlManagerElandControlErrorTest
  ${catkin_LIBRARIES}
  )

add_dependencies(
  MrsUavManagers_ControlManagerElandControlErrorTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# failsafe controlerror test

add_rostest_gtest(MrsUavManagers_ControlManagerFailsafeControlErrorTest
  control_manager/failsafe_control_error/control_manager_failsafe_control_error.test
  control_manager/failsafe_control_error/failsafe_control_error_test.cpp
  )

target_link_libraries(
  MrsUavManagers_ControlManagerFailsafeControlErrorTest
  ${catkin_LIBRARIES}
  )

add_dependencies(
  MrsUavManagers_ControlManagerFailsafeControlErrorTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

# eland service test

add_rostest_gtest(MrsUavManagers_ControlManagerElandServiceTest
  control_manager/eland_service/control_manager_eland_service.test
  control_manager/eland_service/eland_service_test.cpp
  )

target_link_libraries(
  MrsUavManagers_ControlManagerElandServiceTest
  ${catkin_LIBRARIES}
  )

add_dependencies(
  MrsUavManagers_ControlManagerElandServiceTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
