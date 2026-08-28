add_library(apexlab_sim_core
    src/longitudinal_model.cpp
    src/lap_simulation.cpp
    src/nonlinear_planar_model.cpp
    src/optimization_replay.cpp
    src/road_vehicle_model.cpp
    src/planar_model.cpp
    src/scenario.cpp
    src/simulation.cpp
    src/telemetry.cpp
    src/track.cpp
    src/tire_model.cpp
    src/vehicle_parameters.cpp
)

add_library(ApexLab::sim_core ALIAS apexlab_sim_core)

target_include_directories(apexlab_sim_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(apexlab_sim_core PUBLIC cxx_std_20)
apexlab_enable_warnings(apexlab_sim_core)
