add_executable(main main.cpp)
target_link_libraries(main PRIVATE dataloader)
target_link_libraries(main PRIVATE viewer)
target_compile_features(main PRIVATE cxx_std_17)
