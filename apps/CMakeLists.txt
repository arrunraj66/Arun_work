foreach(app_name IN ITEMS mw_info mw_msgdemo mw_pub mw_sub)
  add_executable(${app_name} ${app_name}.cpp)
  target_link_libraries(${app_name} PRIVATE mw::mw mw_warnings)
endforeach()
