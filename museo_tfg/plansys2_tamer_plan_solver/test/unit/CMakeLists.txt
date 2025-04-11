ament_add_gtest(tamer_test tamer_test.cpp)
target_link_libraries(tamer_test ${PROJECT_NAME} dl)
target_compile_definitions(tamer_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")