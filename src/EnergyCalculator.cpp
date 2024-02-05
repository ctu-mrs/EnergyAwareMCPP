#include "EnergyCalculator.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <memory>
#include "utils.hpp"

namespace {
    // NOTE: here PROPELLER_EFFICIENCY contains both motor and propeller efficiency combined.
    // The value is taken is the ratio of a real-world power consumption to a power consumption with ideal motor and propeller calculated by (5), obtained in experiments
    // TODO: move these constants to a config file
    const double AIR_DENSITY = 1.225; // [kg/m^3]
    const double EARTH_GRAVITY = 9.8; //[m/s^2, N/kg]
    const double PROPELLER_EFFICIENCY = 0.391; // Usually from 0.5 to 0.7
    const double RANGE_POWER_CONSUMPTION_COEFF = 1.092; // taken from (17), ratio of power consumption when maximizing the range to power consumption on hover
    const double MOTOR_EFFICIENCY = 1; // Efficiency of the motor (ratio of electric power converted to mechanical)

}

double EnergyCalculator::angle_between_points(std::pair<double, double> p0, std::pair<double, double> p1,
                                              std::pair<double, double> p2) {
    double a = std::pow(p1.first - p0.first, 2) + std::pow(p1.second - p0.second, 2);
    double b = std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2);
    double c = std::pow(p0.first - p2.first, 2) + std::pow(p0.second - p2.second, 2);

    if (((a + b - c) / std::sqrt(4 * a * b) < -1) || ((a + b - c) / std::sqrt(4 * a * b) > 1)) {
        return 0;
    }

    return std::acos((a + b - c) / std::sqrt(4 * a * b));
}

EnergyCalculator::EnergyCalculator(const energy_calculator_config_t &energy_calculator_config,
                                   std::shared_ptr<loggers::SimpleLogger> logger) : m_logger(std::move(logger)),
                                                                                    config(
                                                                                            energy_calculator_config) {


    // induced velocity at hover
    double v_i_h = std::sqrt((config.drone_mass * EARTH_GRAVITY) /
                             (2 * AIR_DENSITY * M_PI * config.propeller_radius * config.propeller_radius *
                              config.number_of_propellers)); // (4)
//    std::cout << v_i_h << std::endl;
    // Power consumption on hover
//  double P_h = (std::sqrt(config.number_of_propellers) * config.drone_mass * EARTH_GRAVITY * v_i_h) / (PROPELLER_EFFICIENCY); // (5)
    P_h = std::sqrt(std::pow(config.drone_mass * EARTH_GRAVITY, 3)) /
          (PROPELLER_EFFICIENCY * config.propeller_radius *
           std::sqrt(2 * AIR_DENSITY * M_PI * config.number_of_propellers));

    // Power consumption when maximizing the range
    P_r = P_h * RANGE_POWER_CONSUMPTION_COEFF; // (17)

    // As soon as the motor efficiency is already taken into account in propeller_motor_efficiency, it is replaced with 1 here
    double P_mot_r = P_r / 1; // (6)

    double P_cell_r = P_mot_r / (config.battery_model.number_of_cells * config.battery_model.cell_capacity); // (14)

    double C_eff_r = config.battery_model.cell_capacity
                     * (config.battery_model.d0
                        + config.battery_model.d1 * P_cell_r
                        + config.battery_model.d2 * std::pow(P_cell_r, 2)
                        + config.battery_model.d3 * std::pow(P_cell_r, 3));


    // Time of flight with the maximum range
    t_r = (C_eff_r * 3.7 * config.battery_model.number_of_cells * 3600) / (P_mot_r);

    // TODO: find out why we should use cm^2 and not m^2 here. For now, formula gives correct result for cm^2, but why in the paper they use cm^2????
    // Here, convert drone area to cm^2 as the parameter is fitted for this value
    double v_r_inv = config.best_speed_model.c0 + config.best_speed_model.c1 * v_i_h +
                     config.best_speed_model.c2 * (config.drone_area * 10000); // (18)

    v_r = v_i_h / v_r_inv;

    m_logger->log_info(
            "ENERGY CALCULATOR: Optimal speed: " + std::to_string(v_r) + "Time of flight: " +
            std::to_string(t_r) + " Hover power consumption: " + std::to_string(P_h) + " Optimal speed power consumption: "  + std::to_string(P_r));
}

turning_properties_t EnergyCalculator::calculate_turning_properties(double angle) const {
    // Calculate the new speed, where x coordinate is the first vector direction
    // i.e. v_y == 0; new_vx == v_x if angle = 180 deg; new_vy == v_x if angle = 90 deg
    angle = std::abs(angle);

    // If angle is extremely close to pi, to avoid any numerical errors due to numbers close to 0, just assume that no
    // Deceleration or acceleration is needed at all
    if (std::abs(angle * 180 / M_PI - 180) < 1) {
        return {v_r, -config.average_acceleration, v_r, config.average_acceleration, 0, 0.0};
    }

    // Updated algorithm version
    double phi = M_PI - angle;
    double phi_2 = (M_PI - angle) / 2;
    double d_x = config.allowed_path_deviation;
    double a_x = config.average_acceleration * std::cos(phi_2);
    double a_y = config.average_acceleration * std::sin(phi_2);

    double dv_x = std::sqrt(2 * d_x * a_x);
    dv_x = std::min(dv_x, std::cos(M_PI_2 - phi) * v_r / 2);

    double dv_y = std::tan(phi_2) * dv_x;
    double vy_m = dv_x / std::tan(phi_2);

    double v_in = vy_m + dv_y;

    // Velocities after the turn
    double v_x = cos(M_PI_2 - phi) * v_r;
    double v_y = sin(M_PI_2 - phi) * v_r;
    double energy = config.drone_mass * (v_r * v_r - v_in * v_in + 0.5 * (v_in * v_in - v_y * v_y + v_x * v_x));

    return {v_in, -a_y, v_in, a_y, energy, vy_m};
}

double EnergyCalculator::calculate_straight_line_energy(double v_in, double a_in, double v_out, double a_out,
                                                        double s_tot) const {
    // Calculate the time and distance travelled during the acceleration and deceleration phases
    double t_acc = std::abs(v_r - v_in) / a_in;
    double s_acc = v_in * t_acc + 0.5 * a_in * std::pow(t_acc, 2);

    double t_dec = std::abs((v_r - v_out) / a_out);
    double s_dec = v_r * t_dec + 0.5 * a_out * std::pow(t_dec, 2);


    if (s_acc + s_dec <= s_tot) {
        m_total_path_time += t_acc + t_dec;
        m_total_path_time += (s_tot - s_acc - s_dec) / v_r;
        return ((s_tot - s_acc - s_dec) / v_r) * P_r +
               calculate_acceleration_energy(v_in, v_r, t_acc) +
               calculate_acceleration_energy(v_out, v_r, t_dec);
    } else {
        return calculate_short_line_energy(v_in, a_in, v_out, a_out, s_tot);
    }
}

double EnergyCalculator::calculate_straight_line_energy(double v_in, double a_in, double v_out, double a_out,
                                                        const std::pair<double, double> &p1,
                                                        const std::pair<double, double> &p2) const {
    return calculate_straight_line_energy(v_in, a_in, v_out, a_out, distance_between_points(p1, p2));
}

double EnergyCalculator::calculate_straight_line_energy_between_turns(const turning_properties_t &turn1,
                                                                      const turning_properties_t &turn2,
                                                                      double s_tot) const {

    double t_acc_slow = std::abs((turn1.v_after - turn1.d_vym) / turn1.a_after);
    double s_acc_slow = turn1.d_vym * t_acc_slow + 0.5 * turn1.a_after * std::pow(t_acc_slow, 2);

    double t_dec_slow = std::abs((turn2.v_before - turn2.d_vym) / turn2.a_before);
    double s_dec_slow = turn2.v_before * t_dec_slow + 0.5 * turn2.a_before * std::pow(t_dec_slow, 2);


    // If the segment is not too short for at least slow acceleration and slow deceleration -- do it
    if (s_acc_slow + s_dec_slow < s_tot) {
        m_total_path_time += t_acc_slow + t_dec_slow;
        double slow_acceleration_energy = calculate_acceleration_energy(turn1.d_vym, turn1.v_after, t_acc_slow);
        slow_acceleration_energy += calculate_acceleration_energy(turn2.d_vym, turn2.v_after, t_dec_slow);

        return calculate_straight_line_energy(turn1.v_after, config.average_acceleration, turn2.v_before,
                                              -config.average_acceleration, s_tot - s_acc_slow - s_dec_slow) +
               slow_acceleration_energy;
    } else {
        // If the UAV can only start the slow deceleration after the slow acceleration
        return calculate_short_line_energy(turn1.d_vym, turn1.a_after, turn2.d_vym, turn2.a_before,
                                           s_tot);
    }
}

double
EnergyCalculator::calculate_short_line_energy(double v_in, double a_in, double v_out, double a_out, double s) const {
    if (s == 0) {
        return 0;
    }

    auto v_sol = std::sqrt(
            (v_in * v_in / a_in + v_out * v_out / std::abs(a_out) + 2 * s) / (1 / a_in + 1 / std::abs(a_out)));

    // If there is no solution as it's impossible to enter and leave the segment with specified speeds having these accelerations
    if (std::isnan(v_sol) || v_sol < v_in || v_sol < v_out || v_sol > v_r) {
        auto middle_speed = (v_out + v_in) / 2;
        m_total_path_time += s / middle_speed;
        return s / middle_speed * (P_h + (P_r - P_h) * middle_speed / v_r);
    }
    double t_acc = (v_sol - v_in) / a_in;
    double s_acc = v_in * t_acc + 0.5 * a_in * t_acc * t_acc;

    double t_dec = std::abs((v_sol - v_out) / a_out);
    double s_dec = v_sol * t_dec + 0.5 * a_out * t_dec * t_dec;


    if (s_acc > 0 && s_dec > 0) {
        m_total_path_time += t_acc + t_dec;
        return calculate_acceleration_energy(v_in, v_sol, t_acc) + calculate_acceleration_energy(v_out, v_sol, t_dec);
    }

    // Should not get here as if v_sol exists, s_acc and s_dec should be positive
    m_logger->log_debug("Warning: Too short segment");
    return 0.0;
}


double EnergyCalculator::calculate_path_energy_consumption(const std::vector<std::pair<double, double>> &path) const {
    if (path.size() < 2) {
        return 0;
    }

    // Filter the path by removing points in the same location
    std::vector<std::pair<double, double>> path_filtered;
    path_filtered.reserve(path.size());
    path_filtered.push_back(path[0]);
    for (size_t i = 1; i < path.size(); ++i) {
        if (distance_between_points(path[i], path[i - 1]) != 0) {
            path_filtered.push_back(path[i]);
        }
    }

    double total_energy = 0;
    std::vector<turning_properties_t> turns;
    turns.push_back({0, 0, 0, config.average_acceleration, config.drone_mass * std::pow(v_r, 2) / 2, 0.0});
    for (size_t i = 1; i + 1 < path_filtered.size(); ++i) {
        double angle = angle_between_points(path_filtered[i - 1], path_filtered[i], path_filtered[i + 1]);
        turns.push_back(calculate_turning_properties(angle));
    }
    turns.push_back({0, -config.average_acceleration, 0, 0, config.drone_mass * std::pow(v_r, 2) / 2, 0.0});

    for (size_t i = 0; i + 1 < path_filtered.size(); ++i) {
        total_energy += turns[i].energy;
        auto energy = calculate_straight_line_energy_between_turns(turns[i], turns[i + 1],
                                                                   distance_between_points(path_filtered[i],
                                                                                           path_filtered[i + 1]));
        total_energy += energy;
    }
    return total_energy;
}


double EnergyCalculator::calculate_acceleration_energy([[maybe_unused]]double v_in, [[maybe_unused]]double v_out, double time) const {
    // For now, just find the average speed as an arithmetic average between v_in and v_out, which is wrong
    // TODO: make this better
//    double avg_speed = (v_in + v_out) / 2;
    double keeping_speed_energy = time * P_h;//(P_h + avg_speed / v_r * (P_r - P_h));
    return keeping_speed_energy;
}
