add_library(viewer STATIC viewer.cpp)

target_link_libraries(viewer PUBLIC Open3D::Open3D)
target_include_directories(viewer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(viewer PRIVATE cxx_std_20)