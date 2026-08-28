add_executable(apexlab-benchmark sim_benchmark.cpp)
target_link_libraries(apexlab-benchmark PRIVATE ApexLab::sim_core)
target_compile_features(apexlab-benchmark PRIVATE cxx_std_20)
apexlab_enable_warnings(apexlab-benchmark)
