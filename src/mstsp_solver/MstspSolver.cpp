#include "mstsp_solver/MstspSolver.h"
#include <utility>
#include "mstsp_solver/Insertion.h"
#include "algorithms.hpp"
#include <algorithm>
#include <list>

vpdd remove_path_heading(const std::vector<point_heading_t<double>> &init) {
    vpdd res;
    for (const auto &p: init) {
        res.emplace_back(p.x, p.y);
    }
    return res;
}

std::vector<point_heading_t<double>> add_path_heading(const vpdd &init, double heading) {
    std::vector<point_heading_t<double>> res;
    for (const auto &p: init) {
        res.emplace_back(p);
        res.back().heading = heading;
    }
    return res;
}

std::vector<point_heading_t<double>> add_path_heading(const vpdd &init, double heading, double alt) {
    auto res = add_path_heading(init, heading);
    for (auto &p: res) {
        p.z = alt;
    }
    return res;
}


namespace mstsp_solver {

    MstspSolver::MstspSolver(SolverConfig config, const std::vector<MapPolygon> &decomposed_polygons,
                             EnergyCalculator energy_calculator,
                             ShortestPathCalculator shortest_path_calculator) : m_logger(
            std::make_shared<loggers::SimpleLogger>()),
                                                                                m_config(std::move(config)),
                                                                                m_energy_calculator(std::move(
                                                                                        energy_calculator)),
                                                                                m_shortest_path_calculator(std::move(
                                                                                        shortest_path_calculator)) {

        for (size_t i = 0; i < decomposed_polygons.size(); ++i) {
            m_target_sets.emplace_back(i, decomposed_polygons[i], m_config.sweeping_step, m_config.wall_distance,
                                       m_energy_calculator,
                                       m_config.rotations_per_cell);
        }
    }


    double MstspSolver::get_path_energy(const std::vector<Target> &path) const {
        if (path.empty()) {
            return 0;
        }
        double energy = 0;

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            energy += path[i].energy_consumption;
            energy += m_energy_calculator.calculate_straight_line_energy(0,
                                                                         m_energy_calculator.get_average_acceleration(),
                                                                         0,
                                                                         -m_energy_calculator.get_average_acceleration(),
                                                                         path[i].end_point,
                                                                         path[i + 1].starting_point);
//            m_logger->log_warn("Energy between points: " + std::to_string(energy));
            // TODO: think if really the shortest path calculation is needed. It works at least in O(N^2) but with caching.
//            auto path_between_polygons = m_shortest_path_calculator.shortest_path_between_points(path[i].end_point, path[i + 1].starting_point);
//            energy += m_energy_calculator.calculate_path_energy_consumption(path_between_polygons);
        }
        energy += path[path.size() - 1].energy_consumption;
        auto a = m_energy_calculator.get_average_acceleration();
        energy += m_energy_calculator.calculate_straight_line_energy(0, a, 0, -a, m_config.starting_point,
                                                                     path[0].starting_point);
        energy += m_energy_calculator.calculate_straight_line_energy(0, a, 0, -a, path[path.size() - 1].end_point,
                                                                     m_config.starting_point);

        return energy;
    }


    double MstspSolver::get_path_cost(const std::vector<Target> &path) const {
        double energy = get_path_energy(path);
        return energy;
    }


    solution_cost_t MstspSolver::get_solution_cost(const _instance_solution_t &solution) const {
        double cost_sum = 0;
        double max_path_cost = 0;

        for (const auto &uav_path: solution) {
            double path_cost = get_path_cost(uav_path);
            cost_sum += path_cost;
            max_path_cost = std::max(max_path_cost, path_cost);
        }

        return {max_path_cost, cost_sum};
    }


    _instance_solution_t MstspSolver::greedy_random() const {
        _instance_solution_t current_solution(m_config.n_uavs);
        auto target_sets = m_target_sets;

        // initial search in close neighborhood
        // - Find all the possible insertions of each target in each path
        // - Calculate the new paths cost of the insertion
        // - Take 1/4 of best insertions
        // - Randomly choose one of them and insert it
        // - Go to first step
        while (!target_sets.empty()) {
            std::vector<Insertion> possible_insertions;
            for (size_t i = 0; i < target_sets.size(); ++i) {
//                std::cout << "i: " <<  i << std::endl;
                for (size_t j = 0; j < m_config.n_uavs; ++j) {
//                    std::cout << "j: " <<  j << std::endl;
                    for (size_t k = 0; k <= current_solution[j].size(); ++k) {
//                        std::cout << "k: " <<  k << std::endl;
                        // TODO: maybe, choose just one insertion from all of the next loop
                        for (size_t target_id = 0; target_id < target_sets[i].targets.size(); ++target_id) {
//                            std::cout << "target_id: " << target_id << std::endl;
                            std::vector<Target> current_route = current_solution[j];
                            current_route.insert(current_route.begin() + static_cast<long>(k),
                                                 target_sets[i].targets[target_id]);
                            double cost = get_path_cost(current_route);
                            possible_insertions.push_back(Insertion{cost, i, target_id, j, k});
                        }
                    }
                }
            }
            std::sort(possible_insertions.begin(), possible_insertions.end(),
                      [](const Insertion &i1, const Insertion &i2) { return i1.solution_cost < i2.solution_cost; });
//            target_sets.erase(target_sets.begin());
            m_logger->log_debug("Possible insertions number " + std::to_string(possible_insertions.size()));
            size_t size_reduced = possible_insertions.size() / 4;
            // Could generate random numbers better, but let it be. We don't need a perfect uniformity
            size_t random = generate_random_number() % (size_reduced + 1);
            if (random >= possible_insertions.size()) {
                random = possible_insertions.size() - 1;
            }
            Insertion chosen_insertion = possible_insertions[random];

//            std::cout << "Chosen target target set: " << target_sets[chosen_insertion.target_set_index].targets[chosen_insertion.target_index].target_set_index << std::endl;

            current_solution[chosen_insertion.uav_index].emplace(current_solution[chosen_insertion.uav_index].begin() +
                                                                 static_cast<long>(chosen_insertion.insertion_index),
                                                                 target_sets[chosen_insertion.target_set_index].targets[chosen_insertion.target_index]);
            target_sets.erase(target_sets.begin() + static_cast<long>(chosen_insertion.target_set_index));
        }
        return current_solution;
    }


    std::vector<std::vector<point_heading_t<double>>>
    MstspSolver::get_drones_paths(const _instance_solution_t &solution) const {
        std::vector<std::vector<point_heading_t<double>>> res;
        int unique_altitude_id = 0;
        for (const auto &i: solution) {
            res.push_back(path_with_heading(i, unique_altitude_id++));
        }
        return res;
    }

    std::vector<point_t> MstspSolver::get_path_from_targets(const std::vector<Target> &targets) const {
        return remove_path_heading(path_with_heading(targets, 10));
    }

    std::vector<point_heading_t<double>>
    MstspSolver::path_with_heading(const std::vector<Target> &targets, int unique_alt_id) const {
        double unique_alt = m_config.sweeping_step;
        if (unique_alt_id % 2 == 0) {
            unique_alt = m_config.sweeping_alt + ((unique_alt_id / 2) + 1) * m_config.unique_alt_step;
        } else {
            unique_alt = m_config.sweeping_alt - ((unique_alt_id - 1) / 2 + 1) * m_config.unique_alt_step;
        }

        std::vector<point_heading_t<double>> res;
        res.emplace_back(m_config.starting_point);
        res.front().z = unique_alt;

        double last_heading = 0;
        for (const auto &target: targets) {
            // Calculate the path from previous target to this one using the shortest path calculator
            auto path_to_target = add_path_heading(
                    m_shortest_path_calculator.shortest_path_between_points({res.back().x, res.back().y},
                                                                            target.starting_point), last_heading,
                    unique_alt);
            path_to_target.pop_back();
            path_to_target.erase(path_to_target.begin());

            res.insert(res.end(), path_to_target.begin(), path_to_target.end());

            auto path = add_path_heading(sweeping(m_target_sets[target.target_set_index].polygon, target.rotation_angle,
                                                  m_config.sweeping_step, m_config.wall_distance, target.first_line_up),
                                         target.rotation_angle, m_config.sweeping_alt);
            res.insert(res.end(), path.begin(), path.end());
            last_heading = target.rotation_angle;
        }
        auto path_to_start = add_path_heading(
                m_shortest_path_calculator.shortest_path_between_points({res.back().x, res.back().y},
                                                                        m_config.starting_point), last_heading,
                unique_alt);
        path_to_start.erase(path_to_start.begin());

        res.insert(res.end(), path_to_start.begin(), path_to_start.end());
        return res;
    }


    final_solution_t MstspSolver::solve() const {
        m_logger->log_info("Solving started");
        _instance_solution_t init_solution = greedy_random();
        size_t nodes = 0;
        for (const auto &uav_path: init_solution) {
            nodes += uav_path.size();
        }

        std::list<_instance_solution_t> tabu_list;
        solution_cost_t best_solution_cost = get_solution_cost(init_solution);

        int best_group = 0;
        solution_cost_t best_neighbourhood_cost = solution_cost_t::max();

        _instance_solution_t best_neighbourhood_solution = init_solution;
        tabu_list.push_back(init_solution);

        int R_T_iterator = m_config.R_T;
        bool stop_criteria = false;
        int iteration = 0;
        int no_improvement_iteration = 0;
        int g1_score = m_config.w0;
        int g2_score = g1_score, g3_score = g1_score, g4_score = g1_score;
        _instance_solution_t final_solution = init_solution;

        while (!stop_criteria) {
            if (iteration % 50 == 0) {
                m_logger->log_debug("==================================================");
                m_logger->log_debug("Iteration: " + std::to_string(iteration));
                m_logger->log_debug("Iteration with no improvement: " + std::to_string(no_improvement_iteration));
                m_logger->log_debug("Best solution cost: " + std::to_string(best_solution_cost.max_path_cost) + ", "
                                    + std::to_string(best_solution_cost.path_cost_sum));
            }
            ++iteration;
            best_neighbourhood_cost = solution_cost_t::max();

            // Reset scores after R_T iterations
            if (++R_T_iterator >= m_config.R_T) {
                g1_score = g2_score = g3_score = g4_score = m_config.w0;
                R_T_iterator = 0;
            }

            for (size_t j = 0; j < nodes; ++j) {
                _instance_solution_t tabu_solution = best_neighbourhood_solution;
                int total_score = g1_score + g2_score + g3_score + g4_score;
                int random = generate_random_number() % total_score;
                if (random < g1_score) {
                    get_g1_solution(tabu_solution);
                } else if (random < (g1_score + g2_score)) {
                    get_g2_solution(tabu_solution);
                } else if (random < (g1_score + g2_score + g3_score)) {
                    get_g3_solution(tabu_solution);
                } else {
                    get_g4_solution(tabu_solution);
                }

                solution_cost_t tabu_solution_cost = get_solution_cost(tabu_solution);
                if (tabu_solution_cost < best_neighbourhood_cost &&
                    std::find(tabu_list.begin(), tabu_list.end(), tabu_solution) == tabu_list.end()) {
                    best_neighbourhood_cost = tabu_solution_cost;
                    best_neighbourhood_solution = tabu_solution;
                    best_group = random;
                }
            }
            ++no_improvement_iteration;
//            std::cout << "Best neighbourhood cost: " << best_neighbourhood_cost << std::endl;
            if (best_neighbourhood_cost < solution_cost_t::max()) {
                if (tabu_list.size() >= nodes / 4) {
                    tabu_list.pop_front();
                }
                tabu_list.push_back(best_neighbourhood_solution);
                if (best_group < g1_score) {
                    g1_score += m_config.p1;
                } else if (best_group < (g1_score + g2_score)) {
                    g2_score += m_config.p1;
                } else if (best_group < (g1_score + g2_score + g3_score)) {
                    g3_score += m_config.p1;
                } else {
                    g4_score += m_config.p1;
                }

                if (best_neighbourhood_cost < best_solution_cost) {
                    final_solution = best_neighbourhood_solution;
                    best_solution_cost = best_neighbourhood_cost;
                    no_improvement_iteration = 0;

                    if (best_group < g1_score) {
                        g1_score += m_config.p2;
                    } else if (best_group < (g1_score + g2_score)) {
                        g2_score += m_config.p2;
                    } else if (best_group < (g1_score + g2_score + g3_score)) {
                        g3_score += m_config.p2;
                    } else {
                        g4_score += m_config.p2;
                    }
                }
            }
            // TODO: check if the commented line ie needed
            //g1_score += m_config.p1;
            if (no_improvement_iteration >= m_config.max_not_improving_iterations) {
                stop_criteria = true;
            }
        }
        return {best_solution_cost.max_path_cost, best_solution_cost.path_cost_sum, get_drones_paths(final_solution)};
    }


    // Random shift intra-inter route
    void MstspSolver::get_g1_solution(_instance_solution_t &solution) const {
        for (size_t i = 0; i <= solution.size(); ++i) {
            // If each UAV visits only 1 or 0 polygons, there is no need (and it will lead to some errors) to continue
            if (i == solution.size()) {
                return;
            }

            if (solution[i].size() >= 2) {
                break;
            }
        }
        size_t routes = solution.size();
        size_t index_a1, index_a2;
        do {
            index_a1 = generate_random_number() % routes;
        } while (solution[index_a1].size() < 2);

        size_t index_c2, index_c1 = generate_random_number() % solution[index_a1].size();

        Target target_to_move = solution[index_a1][index_c1];
        solution[index_a1].erase(solution[index_a1].begin() + static_cast<long>(index_c1));

        if (generate_random_number() % 2 == 0 || solution.size() == 1) { // Shift intra route
            index_a2 = index_a1;
            do {
                index_c2 = generate_random_number() % (solution[index_a1].size() + 1);
            } while (index_c1 == index_c2);
            solution[index_a2].emplace(solution[index_a2].begin() + static_cast<long>(index_c2), target_to_move);
        } else {
            do {
                index_a2 = generate_random_number() % routes;
            } while (index_a2 == index_a1);
            index_c2 = generate_random_number() % (solution[index_a2].size() + 1);
            solution[index_a2].emplace(solution[index_a2].begin() + static_cast<long>(index_c2), target_to_move);
        }
        // Try to rotate the moved target and find the best rotation
        double min_route_cost = get_path_cost(solution[index_a2]);
        Target best_target = target_to_move;
        for (const auto &rotated_target: m_target_sets[target_to_move.target_set_index].targets) {
            solution[index_a2][index_c2] = rotated_target;
            double route_cost = get_path_cost(solution[index_a2]);
            if (route_cost < min_route_cost) {
                min_route_cost = route_cost;
                best_target = rotated_target;
            }
        }
        solution[index_a2][index_c2] = best_target;
    }

    // best shift intra-inter route based on exhaustive search
    void MstspSolver::get_g2_solution(_instance_solution_t &solution) const {
        for (size_t i = 0; i <= solution.size(); ++i) {
            // If each UAV visits only 1 or 0 polygons, there is no need (and it will lead to some errors) to continue
            if (i == solution.size()) {
                return;
            }

            if (solution[i].size() >= 2) {
                break;
            }
        }
        size_t routes = solution.size();
        size_t index_a1;
        do {
            index_a1 = generate_random_number() % routes;
        } while (solution[index_a1].size() < 2);

        size_t index_c1 = generate_random_number() % solution[index_a1].size();

        solution_cost_t best_solution_cost = solution_cost_t::max();
        size_t target_in_target_set_index = 0;

        Target target_to_move = solution[index_a1][index_c1];
        const TargetSet &target_set_to_check = m_target_sets[target_to_move.target_set_index];

        solution[index_a1].erase(solution[index_a1].begin() + static_cast<long>(index_c1));

        for (size_t i = 0; i < solution.size(); ++i) {
            for (size_t j = 0; j <= solution[i].size(); ++j) {
                if (i == index_a1 && j == index_c1) {
                    continue;
                }
                _instance_solution_t intermediate_solution = solution;
                intermediate_solution[i].insert(intermediate_solution[i].begin() + static_cast<long>(j),
                                                target_to_move);
                for (size_t k = 0; k < target_set_to_check.targets.size(); ++k) {
                    intermediate_solution[i][j] = target_set_to_check.targets[k];
                    // TODO: in Franta's code there is something strange here
                    solution_cost_t path_cost = get_solution_cost(intermediate_solution);
                    if (path_cost < best_solution_cost) {
                        best_solution_cost = path_cost;
                        index_a1 = i;
                        index_c1 = j;
                        target_in_target_set_index = k;
                    }
                }
            }
        }
        solution[index_a1].insert(solution[index_a1].begin() + static_cast<long>(index_c1),
                                  m_target_sets[target_to_move.target_set_index].targets[target_in_target_set_index]);


    }

    // best swap intra-inter route based on exhaustive search
    void MstspSolver::get_g3_solution(_instance_solution_t &solution) const {
        for (size_t i = 0; i <= solution.size(); ++i) {
            // If each UAV visits only 1 or 0 polygons, there is no need (and it will lead to some errors) to continue
            if (i == solution.size()) {
                return;
            }

            if (solution[i].size() >= 2) {
                break;
            }
        }
        size_t routes = solution.size();
        size_t index_a1;
        do {
            index_a1 = generate_random_number() % routes;
        } while (solution[index_a1].size() < 2);

        size_t index_c1 = generate_random_number() % solution[index_a1].size();
        size_t index_a2 = index_a1, index_c2 = index_c1;

        auto best_solution_cost = solution_cost_t::max();

        for (size_t i = 0; i < routes; i++) {
            for (size_t j = 0; j < solution[i].size(); ++j) {
                if (i == index_a1 && j == index_c1) {
                    continue;
                }
                auto old_a1_c1 = solution[index_a1][index_c1];
                auto old_i_j = solution[i][j];

                std::swap(solution[index_a1][index_c1], solution[i][j]);
                find_best_targets_for_position(solution, index_a1, index_c1, i, j);
                solution_cost_t solution_cost = get_solution_cost(solution);
                if (solution_cost < best_solution_cost) {
                    best_solution_cost = solution_cost;
                    index_a2 = i;
                    index_c2 = j;
                }
                solution[index_a1][index_c1] = old_a1_c1;
                solution[i][j] = old_i_j;
            }
        }
        if (index_a1 != index_a2 || index_c1 != index_c2) {
            std::swap(solution[index_a1][index_c1], solution[index_a2][index_c2]);
            find_best_targets_for_position(solution, index_a1, index_c1, index_a2, index_c2);
        }
    }


    void MstspSolver::get_g4_solution(_instance_solution_t &solution) const {
        for (size_t i = 0; i <= solution.size(); ++i) {
            // If each UAV visits only 1 or 0 polygons, there is no need (and it will lead to some errors) to continue
            if (i == solution.size()) {
                return;
            }

            if (solution[i].size() >= 2) {
                break;
            }
        }
        size_t routes = solution.size();
        size_t index_a1;
        do {
            index_a1 = generate_random_number() % routes;
        } while (solution[index_a1].size() < 2);

        size_t index_c1 = generate_random_number() % solution[index_a1].size();

        double best_path_cost = get_path_cost(solution[index_a1]);
        Target best_target = solution[index_a1][index_c1];
        const TargetSet &target_to_check = m_target_sets[best_target.target_set_index];
        for (const auto &target: target_to_check.targets) {
            solution[index_a1][index_c1] = target;
            double path_cost = get_path_cost(solution[index_a1]);
            if (path_cost < best_path_cost) {
                best_path_cost = path_cost;
                best_target = target;
            }
        }
        solution[index_a1][index_c1] = best_target;
    }

    void MstspSolver::find_best_targets_for_position(_instance_solution_t &solution, size_t uav1, size_t path_index_1,
                                                     size_t uav2, size_t path_index_2) const {
        solution_cost_t best_cost = solution_cost_t::max();


        // This should definitely change, but to avoid undefined behavior, initialize with 0
        size_t target_1_index = 0, target_2_index = 0;
        const auto &targets_1 = m_target_sets[solution[uav1][path_index_1].target_set_index].targets;
        const auto &targets_2 = m_target_sets[solution[uav2][path_index_2].target_set_index].targets;

        for (size_t i = 0; i < targets_1.size(); ++i) {
            for (size_t j = 0; j < targets_2.size(); ++j) {
                solution[uav1][path_index_1] = targets_1[i];
                solution[uav2][path_index_2] = targets_2[j];
                solution_cost_t path_cost = get_solution_cost(solution);
                if (path_cost < best_cost) {
                    best_cost = path_cost;
                    target_1_index = i;
                    target_2_index = j;
                }
            }
        }
        solution[uav1][path_index_1] = targets_1[target_1_index];
        solution[uav2][path_index_2] = targets_2[target_2_index];
    }
}
