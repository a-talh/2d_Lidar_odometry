add_library(dataloader STATIC dataloader.cpp)

# target_sources(dataloader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/dataloader.cpp)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(dataloader PUBLIC Eigen3::Eigen)


target_include_directories(dataloader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(dataloader PRIVATE cxx_std_20)
