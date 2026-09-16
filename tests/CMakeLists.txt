# One executable per test file. Each is a program that exits non-zero on
# failure; ctest needs nothing more than that.
foreach(test_name IN ITEMS test_version test_message test_transport)
  add_executable(${test_name} ${test_name}.cpp)
  target_link_libraries(${test_name} PRIVATE mw::mw mw_warnings)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
