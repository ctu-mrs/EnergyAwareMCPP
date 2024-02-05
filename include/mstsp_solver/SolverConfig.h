//
// Created by mrs on 28.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_SOLVERCONFIG_H
#define THESIS_TRAJECTORY_GENERATOR_SOLVERCONFIG_H

#include "utils.hpp"
#include <vector>

namespace mstsp_solver {
    struct SolverConfig {
        int rotations_per_cell;
        double sweeping_step;
        point_t starting_point;
        size_t n_uavs;
        double sweeping_alt;
        double unique_alt_step = 1.0; // 1m difference while not sweeping by default
        int max_not_improving_iterations = 0;
        double wall_distance = 0;

        int p1 = 1;
        int p2 = 5;
        int R_T = 20;
        int w0 = 5;

    };
}
#endif //THESIS_TRAJECTORY_GENERATOR_SOLVERCONFIG_H
