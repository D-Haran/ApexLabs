add_executable(apexlab-sim main.cpp replay_export.cpp)
target_link_libraries(apexlab-sim PRIVATE ApexLab::sim_core)
target_compile_features(apexlab-sim PRIVATE cxx_std_20)
apexlab_enable_warnings(apexlab-sim)

add_executable(apexlab-chassis-scenes chassis_scenes.cpp replay_export.cpp)
target_link_libraries(apexlab-chassis-scenes PRIVATE ApexLab::sim_core)
apexlab_enable_warnings(apexlab-chassis-scenes)

add_executable(apexlab-contact-scenes contact_scenes.cpp)
target_link_libraries(apexlab-contact-scenes PRIVATE ApexLab::sim_core)
apexlab_enable_warnings(apexlab-contact-scenes)

add_executable(apexlab-dynamic-bench dynamic_bench.cpp)
target_link_libraries(apexlab-dynamic-bench PRIVATE ApexLab::sim_core)
target_include_directories(apexlab-dynamic-bench PRIVATE ${PROJECT_SOURCE_DIR}/cpp/third-party)
apexlab_enable_warnings(apexlab-dynamic-bench)

add_executable(apexlab-force-audit force_audit.cpp)
target_link_libraries(apexlab-force-audit PRIVATE ApexLab::sim_core)
target_include_directories(apexlab-force-audit PRIVATE ${PROJECT_SOURCE_DIR}/cpp/third-party)
apexlab_enable_warnings(apexlab-force-audit)

add_executable(apexlab-input-audit input_audit.cpp)
target_link_libraries(apexlab-input-audit PRIVATE ApexLab::sim_core)
target_include_directories(apexlab-input-audit PRIVATE ${PROJECT_SOURCE_DIR}/cpp/third-party)
