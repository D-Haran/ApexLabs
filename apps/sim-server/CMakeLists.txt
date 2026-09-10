add_library(apexlab_runtime SHARED runtime.cpp)
target_link_libraries(apexlab_runtime PRIVATE ApexLab::sim_core)
target_include_directories(apexlab_runtime PRIVATE ${PROJECT_SOURCE_DIR}/cpp/third-party)
apexlab_enable_warnings(apexlab_runtime)
