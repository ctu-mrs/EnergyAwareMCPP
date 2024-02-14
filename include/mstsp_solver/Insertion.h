//
// Created by mrs on 28.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_INSERTION_H
#define THESIS_TRAJECTORY_GENERATOR_INSERTION_H

namespace mstsp_solver {

    struct Insertion {
        /*!
         * Struct for storing one possible node insertion during optimization procedure
         */
        double solution_cost; // Solution cost if the insertion is performed
        size_t target_set_index; // Target set of node to insert
        size_t target_index; // Node index in target set
        size_t uav_index; // Index of path to insert the node to
        size_t insertion_index; // Index inside the path to insert the node
    };

    struct InsertionComp {
        bool operator()(const Insertion &i1, const Insertion &i2) const {
            return i1.solution_cost < i2.solution_cost;
        }
    };

}

#endif //THESIS_TRAJECTORY_GENERATOR_INSERTION_H
