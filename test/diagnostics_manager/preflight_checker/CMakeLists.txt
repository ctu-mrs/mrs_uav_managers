ament_add_gtest(test_preflight_checker test.cpp)

target_include_directories(test_preflight_checker PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(test_preflight_checker
  MrsUavManagers_DiagnosticsManager
  rclcpp::rclcpp
  mrs_lib::mrs_lib
  ${mrs_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

target_compile_definitions(test_preflight_checker PRIVATE
  PREFLIGHT_CHECK_CONFIG="${PROJECT_SOURCE_DIR}/config/public/diagnostics_manager/preflight_check.yaml"
  CUSTOM_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/custom_config.yaml"
)
