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

add_executable(apexlab-contact-tests test_contact_vehicle.cpp)
target_link_libraries(apexlab-contact-tests PRIVATE ApexLab::sim_core)
target_compile_definitions(apexlab-contact-tests PRIVATE APEXLAB_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
apexlab_enable_warnings(apexlab-contact-tests)
add_test(NAME apexlab.contact-3d COMMAND apexlab-contact-tests)
add_test(NAME apexlab.contact-cli-smoke COMMAND apexlab-contact-scenes
    ${PROJECT_SOURCE_DIR}/configs/vehicles/mclaren-p1-sprung-approx.json
    ${CMAKE_CURRENT_BINARY_DIR}/contact-scenes .002)

add_executable(apexlab-dynamic-tests test_dynamic_vehicle.cpp)
target_link_libraries(apexlab-dynamic-tests PRIVATE ApexLab::sim_core)
target_compile_definitions(apexlab-dynamic-tests PRIVATE APEXLAB_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
apexlab_enable_warnings(apexlab-dynamic-tests)
add_test(NAME apexlab.dynamic-vehicle COMMAND apexlab-dynamic-tests)

add_executable(apexlab-resource-tests test_resources.cpp)
target_link_libraries(apexlab-resource-tests PRIVATE ApexLab::sim_core)
target_compile_definitions(apexlab-resource-tests PRIVATE APEXLAB_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
apexlab_enable_warnings(apexlab-resource-tests)
add_test(NAME apexlab.resources COMMAND apexlab-resource-tests)

add_test(NAME apexlab.force-audit COMMAND apexlab-force-audit ${PROJECT_SOURCE_DIR}/configs/vehicles/mclaren-p1-dynamic.json)
add_test(NAME apexlab.native-driver COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tools/dynamics -p test_native.py WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_test(NAME apexlab.native-shooting COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tools/dynamics -p test_optimization.py WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

set_tests_properties(apexlab.native-driver apexlab.native-shooting PROPERTIES ENVIRONMENT "APEXLAB_RUNTIME=$<TARGET_FILE:apexlab_runtime>")

# Instrumented/debug builds validate correctness, not release wall-clock budgets.
if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT CMAKE_CXX_FLAGS MATCHES "sanitize")
  set_property(TEST apexlab.native-driver APPEND PROPERTY ENVIRONMENT "APEXLAB_PERFORMANCE_ASSERT=1")
endif()
add_test(NAME apexlab.evaluation-cache COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tools/dynamics -p test_cache.py WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
add_test(NAME apexlab.host-ownership COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tools/dynamics -p test_host.py WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
set_tests_properties(apexlab.host-ownership PROPERTIES ENVIRONMENT "APEXLAB_RUNTIME=$<TARGET_FILE:apexlab_runtime>")
