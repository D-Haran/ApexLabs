add_executable(apexlab-sim main.cpp replay_export.cpp)
target_link_libraries(apexlab-sim PRIVATE ApexLab::sim_core)
target_compile_features(apexlab-sim PRIVATE cxx_std_20)
apexlab_enable_warnings(apexlab-sim)

add_executable(apexlab-chassis-scenes chassis_scenes.cpp replay_export.cpp)
target_link_libraries(apexlab-chassis-scenes PRIVATE ApexLab::sim_core)
apexlab_enable_warnings(apexlab-chassis-scenes)
