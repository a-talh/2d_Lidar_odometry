add_library(viewer STATIC viewer.cpp)
target_link_libraries(viewer PUBLIC Open3D::Open3D)
