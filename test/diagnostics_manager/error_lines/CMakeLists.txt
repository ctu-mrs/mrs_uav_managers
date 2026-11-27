ament_add_gtest(test_error_lines test.cpp)
target_include_directories(test_error_lines PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_error_lines mrs_lib::mrs_lib ${mrs_msgs_TARGETS})
