#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

/* other important includes */
#include <nav_msgs/Odometry.h>

/* user includes */
#include <mrs_lib/subscribe_handler.h>
#include <mrs_msgs/Path.h>
#include <mrs_msgs/PathSrv.h>
#include <std_msgs/String.h>
#include <vector>
#include "EnergyCalculator.h"
#include <thesis_path_generator/GeneratePaths.h>
#include "utils.hpp"
#include "mstsp_solver/MstspSolver.h"
#include "SimpleLogger.h"
#include <thesis_path_generator/CalculateEnergy.h>

namespace path_generation {

/* class TrajectoryGenerator //{ */
    class PathGenerator : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        std::shared_ptr<loggers::SimpleLogger> m_shared_logger;

        /* flags */
        bool m_is_initialized = false;

        /* ros parameters */
        double m_drones_altitude;
        double m_unique_altitude_step;
        energy_calculator_config_t m_energy_config;


        /* other parameters */
        int sequence_counter = 0;
        int m_number_of_rotations;


        // | --------------------- MRS transformer -------------------- |

        // | ---------------------- msg callbacks --------------------- |

        ros::ServiceServer m_calculate_energy_service_server;
        ros::ServiceServer m_generate_paths_service_server;
        ros::Publisher m_path_publisher;

        /*!
         * Callback function for ROS service for paths energy calculation
         * @param req Service request. Contains drone parameters and paths for which energies should be calculated
         * @param res Result message. if calculation is successful contains array of energies for each path
         * @return false of node is not initialized still
         */
        bool callback_calculate_energy(thesis_path_generator::CalculateEnergy::Request &req,
                                       thesis_path_generator::CalculateEnergy::Response &res);

        bool callback_generate_paths(thesis_path_generator::GeneratePaths::Request &req,
                                     thesis_path_generator::GeneratePaths::Response &res);


        [[maybe_unused]] mstsp_solver::final_solution_t
        solve_for_uavs(int n_uavs, const thesis_path_generator::GeneratePaths::Request &req,
                       MapPolygon polygon,
                       const EnergyCalculator &energy_calculator,
                       const ShortestPathCalculator &shortest_path_calculator,
                       std::pair<double, double> gps_transform_origin);


        /*!
         * Generate path for one drone flying in a simulation
         * @param points_to_visit Points that need to be visited in the appropriate order
         * @param max_distance_between_points Max distance between two consecutive points. If initial distance between
         * consecutive points is larger, new ones will be added
         * @return Path that can be sent to the follower or trajectory generator to follow it
         */
        mrs_msgs::Path
        _generate_path_for_simulation_one_drone(const std::vector<point_heading_t<double>> &points_to_visit,
                                                point_t gps_transform_origin,
                                                double optimal_speed = 1,
                                                double horizontal_acceleration = 2.0);

        /*!
         * Generate paths with max energy not more than max_energy_bound. Number of produced paths is greater or equal to the number of UAVs
         * @tparam F callable type for generating paths with the specified number of uavs. (int) -> mstsp_solver::final_solution_t
         * @param max_energy_bound maximum energy of one path in Joules
         * @param n_uavs Number of uavs. There will be no less paths than this number
         * @param f Function that generates the specified number of paths
         * @return Solution to the problem
         */
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
                    ROS_WARN(
                            "[PathGenerator]: could not generate paths to satisfy the upper bound on energy consumption...");
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
    };


}  // namespace trajectory_generatiion
