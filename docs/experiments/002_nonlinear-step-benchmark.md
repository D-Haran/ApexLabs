# Nonlinear four-tire RK4 step benchmark

Recorded on Apple M2 Pro (`arm64`) using Apple clang 17.0.0, CMake Release mode
(`-O3 -DNDEBUG` from the generator defaults), `std::chrono::steady_clock`, 2,000 warmup steps, and
100,000 timed RK4 steps. The representative input is 20 m/s, 0.025 rad steer, and 0.25 throttle.

```text
workload=nonlinear-four-tire-rk4 iterations=100000 warmup_iterations=2000
elapsed_s=2.019 ns_per_step=20185.462 solver_mean_iterations=22.248
solver_median_iterations=22 solver_max_iterations=23
```

For workload context, the same executable measured the preserved linear planar RK4 workload at
`194.684 ns/step` and longitudinal RK4 at `101.122 ns/step` in that run. These are different
workloads: each nonlinear RK4 stage performs a converged four-tire force/load iteration, so the
numbers must not be interpreted as an implementation-only speed regression. Checksums are emitted
to keep the timed work observable.
