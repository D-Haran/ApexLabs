add_executable(apexlab-tests test_main.cpp)
target_link_libraries(apexlab-tests PRIVATE ApexLab::sim_core)
target_compile_features(apexlab-tests PRIVATE cxx_std_20)
target_compile_definitions(apexlab-tests PRIVATE APEXLAB_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
apexlab_enable_warnings(apexlab-tests)

add_test(NAME apexlab.unit-and-integration COMMAND apexlab-tests)

add_test(
    NAME apexlab.cli-smoke
    COMMAND apexlab-sim
        --vehicle ${PROJECT_SOURCE_DIR}/configs/vehicles/generic_performance_car.json
        --output ${CMAKE_CURRENT_BINARY_DIR}/cli-smoke.csv
        --duration 0.1
        --dt 0.01
        --integrator rk4
        --throttle 0.25
)

find_package(Python3 3.10 COMPONENTS Interpreter REQUIRED)
add_test(
    NAME apexlab.track-importer
    COMMAND ${Python3_EXECUTABLE} -m unittest test_import_osm_track.py
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tools/tracks
)

add_test(
    NAME apexlab.asset-inspector
    COMMAND ${Python3_EXECUTABLE} -m unittest test_inspect_gltf.py
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tools/assets
)

add_test(
    NAME apexlab.planar-cli-smoke
    COMMAND apexlab-sim
        --vehicle ${PROJECT_SOURCE_DIR}/configs/vehicles/planar_neutral.json
        --scenario ${PROJECT_SOURCE_DIR}/configs/scenarios/step_steer.json
        --output ${CMAKE_CURRENT_BINARY_DIR}/planar-cli-smoke.csv
)

add_test(
    NAME apexlab.nonlinear-planar-cli-smoke
    COMMAND apexlab-sim
        --vehicle ${PROJECT_SOURCE_DIR}/configs/vehicles/generic_nonlinear_performance_car.json
        --scenario ${PROJECT_SOURCE_DIR}/configs/scenarios/ramp_steer.json
        --output ${CMAKE_CURRENT_BINARY_DIR}/nonlinear-planar-cli-smoke.csv
)

add_test(
    NAME apexlab.p1-config-smoke
    COMMAND apexlab-sim
        --vehicle ${PROJECT_SOURCE_DIR}/configs/vehicles/mclaren-p1-approx.json
        --output ${CMAKE_CURRENT_BINARY_DIR}/p1-config-smoke.csv
        --duration 0.1
        --dt 0.01
        --integrator rk4
        --throttle 0.1
)

add_test(
    NAME apexlab.lap-cli-smoke
    COMMAND apexlab-sim
        --vehicle ${PROJECT_SOURCE_DIR}/configs/vehicles/generic_nonlinear_performance_car.json
        --track ${PROJECT_SOURCE_DIR}/configs/tracks/technical_test_circuit.json
        --output ${CMAKE_CURRENT_BINARY_DIR}/lap-cli-smoke.csv
        --warmup-laps 0
        --laps 1
        --dt 0.01
        --controller-dt 0.02
)

add_test(
    NAME apexlab.optimization-replay-smoke
    COMMAND apexlab-sim
        --vehicle ${PROJECT_SOURCE_DIR}/configs/vehicles/generic_nonlinear_performance_car.json
        --track ${PROJECT_SOURCE_DIR}/configs/tracks/technical_test_circuit.json
        --optimization-profile ${PROJECT_SOURCE_DIR}/data/generated/optimization/technical-fixed/fixed-line-n100.csv
        --replay-diagnostics ${CMAKE_CURRENT_BINARY_DIR}/optimization-replay-smoke.json
        --output ${CMAKE_CURRENT_BINARY_DIR}/optimization-replay-smoke.csv
        --warmup-laps 0
        --laps 1
        --dt 0.01
        --maximum-lap-duration 120
)

add_executable(apexlab-road-tests test_road_vehicle.cpp)
target_link_libraries(apexlab-road-tests PRIVATE ApexLab::sim_core)
target_compile_definitions(apexlab-road-tests PRIVATE APEXLAB_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
apexlab_enable_warnings(apexlab-road-tests)
add_test(NAME apexlab.road-sprung-body COMMAND apexlab-road-tests)
