add_library(registration STATIC registration.cpp)

target_link_libraries(registration PUBLIC Open3D::Open3D)
target_include_directories(registration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(registration PRIVATE cxx_std_20)