find_package(rostest REQUIRED)

# takeoff test

add_rostest_gtest(MrsUavManagers_UavManagerTakeoffTest
  uav_manager/takeoff/uav_manager_takeoff.test
  uav_manager/takeoff/test.cpp
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
  uav_manager/landing/test.cpp
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

# goto test

add_rostest_gtest(MrsUavManagers_ControlManagerGotoTest
  control_manager/goto/control_manager_goto.test
  control_manager/goto/test.cpp
  )

target_link_libraries(
  MrsUavManagers_ControlManagerGotoTest
  ${catkin_LIBRARIES}
  )

add_dependencies(
  MrsUavManagers_ControlManagerGotoTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
