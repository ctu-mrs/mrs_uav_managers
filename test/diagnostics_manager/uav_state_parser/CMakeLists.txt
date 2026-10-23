ament_add_gtest(test_uav_state_parser test.cpp)
target_include_directories(test_uav_state_parser PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_uav_state_parser ${mrs_msgs_TARGETS})
