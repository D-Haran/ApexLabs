# Foundation RK4 step benchmark

Date: 2026-09-19

## Question

Can the initial longitudinal RK4 implementation execute comfortably faster than its intended
200 Hz real-time rate on the development machine?

## Hypothesis

The dependency-free two-state model should take substantially less than the 5 ms budget available
per step at 200 Hz. This is a smoke-level baseline, not an optimization target.

## Setup and method

- Apple M2 Pro (`arm64`), macOS 26.6
- Apple Clang 17.0.0
- CMake `Release` build
- 2,000,000 sequential RK4 steps per process
- Fixed `dt = 0.005 s`, generic assumed vehicle parameters, constant throttle
- Wall time from `std::chrono::steady_clock`
- A final-state checksum is printed so the integration loop remains observable

Command:

```sh
./build/cpp/benchmarks/apexlab-benchmark
```

## Results

Two consecutive runs produced:

| Run | elapsed (s) | ns/RK4 step | checksum |
|---:|---:|---:|---:|
| 1 | 0.185 | 92.713 | 1029590.708 |
| 2 | 0.181 | 90.320 | 1029590.708 |

## Interpretation

This narrow model is far below a 5 ms single-step budget on this machine. The matching checksum
also provides a basic guard against a removed loop, although it is not a substitute for inspecting
optimized machine code or using a dedicated benchmark framework.

## Limitations

These are local wall-clock measurements, not portable performance claims. The benchmark excludes
configuration parsing, telemetry allocation/serialization, control scheduling, transport, and
future tire/track work. Frequency scaling, background load, compiler version, and build flags can
change the result. Re-run it after meaningful model changes rather than treating these numbers as
a permanent project metric.
