# Milestone 2 planar RK4 step benchmark

Date: 2026-09-19

## Question

What is the local cost of one six-state planar bicycle-model RK4 step, and does it remain well
below the 5 ms budget implied by a 200 Hz simulation loop?

## Hypothesis

The added axle slip, virtual tire dispatch, trigonometry, coordinate transform, and six-state RK4
work will cost more than the two-state longitudinal step but remain far below 5 ms.

## Configuration and method

- Apple M2 Pro (`arm64`), macOS 26.6
- Apple Clang 17.0.0
- CMake `Release` build
- `std::chrono::steady_clock` wall timing
- 20,000 untimed warmup steps followed by 2,000,000 timed sequential steps per workload
- fixed `dt = 0.005 s`, constant controls, final-state checksum retained
- dependency-free executable `build/cpp/benchmarks/apexlab-benchmark`

The benchmark emits the preserved longitudinal workload and the planar workload separately. Their
numbers are not treated as equivalent work: the planar step evaluates four six-state derivatives,
two axle slip angles/tire laws, longitudinal forces, body force resolution, and world kinematics.

## Results

Two consecutive process runs produced:

| run | workload | elapsed (s) | ns/RK4 step | checksum |
|---:|---|---:|---:|---:|
| 1 | longitudinal | 0.191 | 95.285 | 1029590.708 |
| 1 | planar bicycle | 0.383 | 191.397 | 1644.272 |
| 2 | longitudinal | 0.190 | 95.189 | 1029590.708 |
| 2 | planar bicycle | 0.404 | 201.840 | 1644.272 |

## Interpretation

The measured planar step was 191–202 ns on this machine, around twice this executable's
longitudinal workload and more than four orders of magnitude below 5 ms. This is headroom evidence
for the current isolated physics step, not a real-time system guarantee.

## Limitations

The loop excludes JSON parsing, scenario dispatch, telemetry allocation/serialization, plotting,
transport, rendering, per-wheel work, and future nonlinear tires. Compiler optimization, CPU
frequency, background load, and hardware affect the result. The long sequential trajectory is a
stable synthetic workload, not a physically meaningful driving experiment.
