
find_package(Catch2 REQUIRED CONFIG)

add_executable(cbag_polygon_test
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/geo_index.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/interval_data.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/point_data.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/polygon_45_data.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/polygon_45_set_vertex.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/polygon_90_data.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/polygon_data.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/polygon_set_data.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/rectangle_data.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cbag/polygon/transformation.cpp
  )

target_include_directories(cbag_polygon_test
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

target_link_libraries(cbag_polygon_test
  PUBLIC
  cbag_polygon
  yaml-cpp
  Catch2::Catch2
  PRIVATE
  fmt::fmt
  )
