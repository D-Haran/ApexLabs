#pragma once
#include "apexlab/lap_simulation.hpp"

// Presentation export only: serialize forces already evaluated by the simulator.
void write_replay_data(const std::filesystem::path& directory,
                       const apexlab::LapSimulationResult& result,
                       const apexlab::PeriodicTrack& track,
                       const apexlab::LapSimulationOptions& options);
