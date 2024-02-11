#include "MapPolygon.hpp"
#include "EnergyCalculator.h"
#include "algorithms.hpp"
#include "ShortestPathCalculator.hpp"
#include "mstsp_solver/SolverConfig.h"
#include "mstsp_solver/MstspSolver.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include "SimpleLogger.h"
#include "utils.hpp"
#include <iomanip>

// Struct defining full algorithm config in one place
struct algorithm_config_t {
    energy_calculator_config_t energy_calculator_config;
    int number_of_rotations;
    bool points_in_lat_lon;
    std::pair<double, double> lat_lon_origin;
    std::string fly_zone_points_file;
    std::vector<std::string> no_fly_zone_points_files;
    int number_of_drones;
    double sweeping_step;
    decomposition_type_t decomposition_type;
    int min_sub_polygons_per_uav;
    std::pair<double, double> start_pos;

    int rotations_per_cell;
    int no_improvement_cycles_before_stop;
    double max_single_path_energy;
};

/*!
 * Parse algorithm configuration from YAML object into algorithm_config_t object
 * @param config YAML node read from the configuration file
 */
algorithm_config_t parse_algorithm_config(const YAML::Node &config);

/*!
 * Check whether the algorithm configuration is valid (i.e. contains all the required fields and dependencies)
 */
bool algorithm_config_is_valid(const YAML::Node &config);

/*!
 * Generate paths with max energy not more than max_energy_bound. Number of produced paths is greater or equal to the number of UAVs
 * @tparam F callable type for generating paths with the specified number of uavs. (int) -> mstsp_solver::final_solution_t
 * @param max_energy_bound maximum energy of one path in Joules
 * @param n_uavs Number of uavs. There will be no less paths than this number
 * @param f Function that generates the specified number of paths
 * @return Solution to the problem
 */
template<typename F>
mstsp_solver::final_solution_t generate_with_constraints(double max_energy_bound, unsigned int n_uavs, F f);

/*!
 * Read points from CSV file into vector of points
 */
std::vector<point_t> read_points_from_csv(const std::string &filename);

/*!
 * Write points into a CSV file with two columns
 */
void write_polygon_into_csv(const std::vector<point_heading_t<double>> &path, const std::string &filename);

/*!
 * Solve the algorithm for the specific number of UAV flights
 * @param n_uavs Number of UAVs -- exact number of paths to be generated
 * @param algorithm_config Algorithm configuration
 * @param polygon Polygon to solve for
 * @param energy_calculator Initialized energy calculator for the specific UAV
 * @param shortest_path_calculator Initialized shorted path calculator
 * @param logger Logger to log output
 */
mstsp_solver::final_solution_t
solve_for_uavs(int n_uavs, const algorithm_config_t &algorithm_config,
               MapPolygon polygon,
               const EnergyCalculator &energy_calculator,
               const ShortestPathCalculator &shortest_path_calculator,
               std::shared_ptr<loggers::SimpleLogger> &logger);


int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Error. Usage: coverage_mission_planner <algorithm_configuration>.yaml" << std::endl;
        return -1;
    }
    YAML::Node algorithm_config_node = YAML::LoadFile(argv[1]);
    if (!algorithm_config_is_valid(algorithm_config_node)) {
        std::cerr << "Algorithm config is not complete. Exiting..." << std::endl;
        return -1;
    }
    algorithm_config_t algorithm_config;
    try {
        algorithm_config = parse_algorithm_config(algorithm_config_node);
    } catch (const YAML::Exception &e) {
        std::cout << "Error while parsing YAML configuration file: " << e.what() << std::endl;
        return -1;
    }
    auto fly_zone = read_points_from_csv(algorithm_config.fly_zone_points_file);
    if (fly_zone.empty()) {
        std::cout << "Error: fly zone points file is either empty or of wrong format" << std::endl;
        return -1;
    }
    std::cout << "Fly zone: " << std::endl;
    for (const auto &p: fly_zone) {
        std::cout << p.first << ", " << p.second << std::endl;
    }

    std::vector<std::vector<point_t>> no_fly_zones;
    for (const auto &s: algorithm_config.no_fly_zone_points_files) {
        auto no_fly_zone = read_points_from_csv(s);
        if (no_fly_zone.empty()) {
            std::cout << "Error: no fly zone file " << s << " is either empty or of a wrong format" << std::endl;
            return -1;
        }
    }

    // Create a logger to log everything directly into stdout
    auto shared_logger = std::make_shared<loggers::SimpleLogger>();
    EnergyCalculator energy_calculator{algorithm_config.energy_calculator_config, shared_logger};
    std::cout << "Energy calculator created. Optimal speed: " << energy_calculator.get_optimal_speed() << std::endl;

    // Initialize polygon and transform all the point into meters
    MapPolygon polygon;
    if (algorithm_config.points_in_lat_lon) {
        polygon = MapPolygon(fly_zone, no_fly_zones, algorithm_config.lat_lon_origin);
    } else {
        polygon = MapPolygon(fly_zone, no_fly_zones);
    }
    // Decompose the polygon
    ShortestPathCalculator shortest_path_calculator(polygon);

    mstsp_solver::final_solution_t best_solution;
    try {
        auto f = [&](int n) {
            return solve_for_uavs(n, algorithm_config, polygon, energy_calculator, shortest_path_calculator,
                                  shared_logger);
        };
        best_solution = generate_with_constraints(algorithm_config.max_single_path_energy * 3600,
                                                  algorithm_config.number_of_drones, f);
    } catch (const polygon_decomposition_error &e) {
        std::cout << "Error while decomposing the polygon" << std::endl;
        return -1;
    } catch (const std::runtime_error &e) {
        std::cout << "Error while solving for polygons: " << e.what();
        return -1;
    }

    std::cout << "Writing output paths into files" << std::endl;
    auto best_paths = best_solution.paths;

    // If initial paths were read in lat_lon coordinates, write the output paths in the same way
    if (algorithm_config.points_in_lat_lon) {
        for (auto &path: best_paths) {
            for (auto &p: path) {
                auto lat_lon_p = meters_to_gps_coordinates({p.x, p.y}, algorithm_config.lat_lon_origin);
                p.x = lat_lon_p.first;
                p.y = lat_lon_p.second;
            }}
    }

    for (size_t i = 0; i < best_paths.size(); ++i) {
        write_polygon_into_csv(best_paths[i], "path_" + std::to_string(i) + ".csv");
    }


    return 0;
}


template<typename F>
[[maybe_unused]] mstsp_solver::final_solution_t
generate_with_constraints(double max_energy_bound, unsigned int n_uavs, F f) {
    // Generate the initial solution that can be optimized after
    mstsp_solver::final_solution_t solution = f(n_uavs);
    if (solution.max_path_energy < max_energy_bound) {
        return solution;
    }

    auto current_n_uavs = n_uavs;
    int iteration = 0;
    while (solution.max_path_energy > max_energy_bound) {
        // Stop if too many iterations are already done. TODO: remove the hardcoded value from here
        if (++iteration > 10) {
            std::cout << "Could not generate paths to satisfy the upper bound on energy consumption..." << std::endl;
            return solution;
        }
        // if the energy consumption is divided well, this should be enough
        unsigned int updated_n_uavs = std::ceil(solution.path_energies_sum / max_energy_bound);

        // if the needed number of UAVs is estimated to be smaller than on previous step, make it larger
        if (updated_n_uavs <= current_n_uavs) {
            updated_n_uavs = current_n_uavs + 1;
        }
        current_n_uavs = updated_n_uavs;
        solution = f(current_n_uavs);
    }
    return solution;

}

std::vector<point_t> read_points_from_csv(const std::string &filename) {
    std::ifstream is(filename);

    if (!is.is_open()) {
        std::cerr << "Error opening file " << filename << std::endl;
        return {};
    }

    std::vector<point_t> points;

    // Read the CSV file line by line
    std::string line;
    while (std::getline(is, line)) {
        std::istringstream iss(line);
        std::string x_str, y_str;

        // Split the line into latitude and longitude
        if (std::getline(iss, x_str, ',') && std::getline(iss, y_str, ',')) {
            // Add the pair to the vector
            points.emplace_back(std::stod(x_str), std::stod(y_str));
        } else {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
    }

    is.close();
    // Make sure that the first point is always the same as the last one
    if (not points.empty() and points[0] != points[points.size() - 1]) {
        points.push_back(points[0]);
    }
    return points;
}

void write_polygon_into_csv(const std::vector<point_heading_t<double>> &path, const std::string &filename) {
    std::ofstream of{filename};
    of << std::setprecision(10);
    for (const auto &p: path) {
        of << p.x << ", " << p.y << std::endl;
    }
    of.close();
}


bool algorithm_config_is_valid(const YAML::Node &config) {
    static const char *required_fields[] = {"battery_model", "best_speed_model", "drone_mass", "drone_area",
                                            "average_acceleration",
                                            "propeller_radius", "number_of_propellers", "allowed_path_deviation",
                                            "number_of_rotations", "points_in_lat_lon", "fly_zone_filename",
                                            "number_of_drones", "sweeping_step", "decomposition_method",
                                            "min_sub_polygons_per_uav", "start_x", "start_y", "rotations_per_cell",
                                            "no_improvement_cycles_before_stop", "max_single_path_energy"};
    for (const auto field: required_fields) {
        if (!config[field]) {
            std::cerr
                    << "Error: not all the required fields are present in the config file. Check the example configuration to see what you may have missed"
                    << std::endl;
            return false;
        }
    }

    auto battery_model_config = config["battery_model"];
    if (!battery_model_config["cell_capacity"] ||
        !battery_model_config["number_of_cells"] ||
        !battery_model_config["d0"] ||
        !battery_model_config["d1"] ||
        !battery_model_config["d2"] ||
        !battery_model_config["d2"]) {

        std::cerr << "Error: battery_model node does not contain all the required parameters" << std::endl;
        return false;
    }

    auto best_speed_model_config = config["best_speed_model"];
    if (!best_speed_model_config["c0"] ||
        !best_speed_model_config["c1"] ||
        !best_speed_model_config["c2"]) {

        std::cerr << "Error: best_speed_model node does not contain all the required parameters" << std::endl;
        return false;
    }

    if (config["points_in_lat_lon"].as<bool>()) {
        if (!config["latitude_origin"] ||
            !config["longitude_origin"]) {
            std::cerr << "Error: points_in_lat_lon is set to True, but no latitude and longitude origins are specified"
                      << std::endl;
            return false;
        }
    }

    if (config["decomposition_method"].as<int>() >= decomposition_type_t::DECOMPOSITION_TYPES_NUMBER) {
        std::cerr << "Error: invalid decomposition type" << std::endl;
        return false;
    }

    return true;
}

algorithm_config_t parse_algorithm_config(const YAML::Node &config) {
    algorithm_config_t algorithm_config;
    algorithm_config.energy_calculator_config.drone_mass = config["drone_mass"].as<double>();
    algorithm_config.energy_calculator_config.drone_area = config["drone_area"].as<double>();
    algorithm_config.energy_calculator_config.average_acceleration = config["average_acceleration"].as<double>();
    algorithm_config.energy_calculator_config.propeller_radius = config["propeller_radius"].as<double>();
    algorithm_config.energy_calculator_config.number_of_propellers = config["number_of_propellers"].as<int>();
    algorithm_config.energy_calculator_config.allowed_path_deviation = config["allowed_path_deviation"].as<double>();
    algorithm_config.number_of_rotations = config["number_of_rotations"].as<int>();

    auto battery_model_config = config["battery_model"];
    algorithm_config.energy_calculator_config.battery_model.cell_capacity = battery_model_config["cell_capacity"].as<double>();
    algorithm_config.energy_calculator_config.battery_model.number_of_cells = battery_model_config["number_of_cells"].as<int>();
    algorithm_config.energy_calculator_config.battery_model.d0 = battery_model_config["d0"].as<double>();
    algorithm_config.energy_calculator_config.battery_model.d1 = battery_model_config["d1"].as<double>();
    algorithm_config.energy_calculator_config.battery_model.d2 = battery_model_config["d2"].as<double>();
    algorithm_config.energy_calculator_config.battery_model.d3 = battery_model_config["d3"].as<double>();

    auto best_speed_model_config = config["best_speed_model"];
    algorithm_config.energy_calculator_config.best_speed_model.c0 = best_speed_model_config["c0"].as<double>();
    algorithm_config.energy_calculator_config.best_speed_model.c1 = best_speed_model_config["c1"].as<double>();
    algorithm_config.energy_calculator_config.best_speed_model.c2 = best_speed_model_config["c2"].as<double>();

    algorithm_config.points_in_lat_lon = config["points_in_lat_lon"].as<bool>();
    if (algorithm_config.points_in_lat_lon) {
        algorithm_config.lat_lon_origin = {config["latitude_origin"].as<double>(),
                                           config["longitude_origin"].as<double>()};
    }

    if (config["no_fly_zones_filenames"]) {
        for (const auto &node: config["no_fly_zones_filename"]) {
            algorithm_config.no_fly_zone_points_files.emplace_back(node.as<std::string>());
        };
    }

    algorithm_config.fly_zone_points_file = config["fly_zone_filename"].as<std::string>();

    algorithm_config.number_of_drones = config["number_of_drones"].as<int>();
    algorithm_config.sweeping_step = config["sweeping_step"].as<int>();
    algorithm_config.decomposition_type = static_cast<decomposition_type_t>(config["decomposition_method"].as<int>());
    algorithm_config.min_sub_polygons_per_uav = config["min_sub_polygons_per_uav"].as<int>();

    algorithm_config.start_pos = {config["start_x"].as<double>(), config["start_y"].as<double>()};
    algorithm_config.rotations_per_cell = config["rotations_per_cell"].as<int>();
    algorithm_config.no_improvement_cycles_before_stop = config["no_improvement_cycles_before_stop"].as<int>();
    algorithm_config.max_single_path_energy = config["max_single_path_energy"].as<double>();

    return algorithm_config;
}

[[maybe_unused]] mstsp_solver::final_solution_t
solve_for_uavs(int n_uavs, const algorithm_config_t &algorithm_config,
               MapPolygon polygon,
               const EnergyCalculator &energy_calculator,
               const ShortestPathCalculator &shortest_path_calculator,
               std::shared_ptr<loggers::SimpleLogger> &logger) {
    auto init_polygon = polygon;

    auto best_initial_rotations = n_best_init_decomp_angles(polygon, algorithm_config.number_of_rotations,
                                                            algorithm_config.decomposition_type);

    std::cout << "Calculated best rotations: " << std::endl;
    for (const auto &rot: best_initial_rotations) {
        std::cout << rot << std::endl;
    }

    // Run algorithm for each rotation and save the best result
    double best_solution_cost = std::numeric_limits<double>::max();
    mstsp_solver::final_solution_t best_solution;
    for (const auto &rotation: best_initial_rotations) {
        // Decompose polygon using initial rotation
        polygon = init_polygon.rotated(rotation);
        std::vector<MapPolygon> polygons_decomposed;
        polygons_decomposed = trapezoidal_decomposition(polygon,
                                                        static_cast<decomposition_type_t>(algorithm_config.decomposition_type));

        std::cout << "Polygon decomposed. Decomposed polygons: " << std::endl;
        for (const auto &p: polygons_decomposed) {
            std::cout << "Decomposed sub polygon area: " << p.area() << std::endl;
        }

        // Divide large polygons into smaller ones to meet the constraint on the lowest number of sub polygons
        std::cout << "Dividing large polygons into smaller ones" << std::endl;
        std::vector<MapPolygon> polygons_divided;
        try {
            polygons_divided = split_into_number(polygons_decomposed,
                                                 static_cast<size_t>(n_uavs) *
                                                 algorithm_config.min_sub_polygons_per_uav);
        } catch (std::runtime_error &e) {
            std::cout << "ERROR while dividing polygon: " << e.what() << std::endl;
            return best_solution;
        }

        for (auto &p: polygons_divided) {
            p = p.rotated(-rotation);
        }
        polygons_decomposed = polygons_divided;
        std::cout << "Divided large polygons into smaller ones" << std::endl;

        // Create the configuration for MSTSP solver
        auto starting_point = algorithm_config.points_in_lat_lon ? gps_coordinates_to_meters(algorithm_config.start_pos,
                                                                                             algorithm_config.lat_lon_origin)
                                                                 :
                              algorithm_config.start_pos;
        mstsp_solver::SolverConfig solver_config{algorithm_config.rotations_per_cell, algorithm_config.sweeping_step,
                                                 starting_point,
                                                 static_cast<size_t>(n_uavs), 0,
                                                 0,
                                                 algorithm_config.no_improvement_cycles_before_stop};
        solver_config.wall_distance = algorithm_config.sweeping_step / 2;
        mstsp_solver::MstspSolver solver(solver_config, polygons_decomposed, energy_calculator,
                                         shortest_path_calculator);
        solver.set_logger(logger);

        auto solver_res = solver.solve();

        // Change the best solution if the current one is better
        if (solver_res.max_path_energy < best_solution_cost) {
            best_solution_cost = solver_res.max_path_energy;
            best_solution = solver_res;
            std::cout << "Best solution rotation: " << rotation / M_PI * 180 << std::endl;
        }
    }
    return best_solution;
}

