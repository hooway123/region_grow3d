add_compile_options(-std=c++11)
add_library(region_grow SHARED region_grow.cpp)
set_target_properties(region_grow PROPERTIES PUBLIC_HEADER "region_grow.h")
target_link_libraries(region_grow)

install(TARGETS region_grow
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/region_grow)

#if(BUILD_TESTING)
add_executable(rgDemo rg_demo.cpp)
target_link_libraries(rgDemo region_grow)
add_test(rgDemo ${CMAKE_BINARY_DIR}/bin/rgDemo)
#endif()
