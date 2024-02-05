
#ifndef ENERGY_CALCULATOR_H
#define ENERGY_CALCULATOR_H

#include <utility>
#include <vector>
#include "SimpleLogger.h"
#include <memory>

struct battery_model_t {
    double cell_capacity;
    int number_of_cells;

    // Coefficients for the equation https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9691840(21)
    // k = C_eff / C = d_0 + d1 * P_cell + d2 * P_cell^2 + d3 * P_cell^3
    double d0;
    double d1;
    double d2;
    double d3;
};


/*!
 * Constants for determination of the most efficient movement speed.
 * Based on the equation https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9691840 (18)
 */
struct best_speed_model_t {
    double c0;
    double c1;
    double c2;
};


struct turning_properties_t {
    double v_before;
    double a_before;
    double v_after;
    double a_after;
    double energy;
    double d_vym;
};

/*! 
 * Structure with pre-calculated constants for the accurate calculation of the energy consumption 
 * */
struct energy_calculator_config_t {
    battery_model_t battery_model;
    best_speed_model_t best_speed_model;
    double drone_mass; // [kg]
    double drone_area; // [m^2]
    double average_acceleration; // [m/s^2]
    double propeller_radius; // [m]
    int number_of_propellers;
    double allowed_path_deviation;
};


class EnergyCalculator {
private:
    std::shared_ptr<loggers::SimpleLogger> m_logger;

    energy_calculator_config_t config;
    double v_r; // speed at which the drone can cover the longest range [m/s]
    double t_r; // Time during thich the drone can fly with the speed v_r [s]
    double P_r; // Power consumption during movement with the speed v_r
    double P_h; // Power consumption during hover

    mutable double m_total_path_time = 0.0;


    /*!
     * Calculate the energy spent on turning manuver including the deceleration and acceleration
     *
     * @param angle Turning angle [rad]
     * @return Properties of the turn by angle
     */
    [[nodiscard]] turning_properties_t calculate_turning_properties(double angle) const;

public:

    /*!
     * Calculate the angle between segment (p1, p2) and segment (p2, p3) in radians
     * @param p1: coordinates of a start point
     * @param p2: coordinates of middle point
     * @param p3: coordinates of the third point
     * @return The angle in radians in range (0..PI)
     */
    [[nodiscard]] static double
    angle_between_points(std::pair<double, double> p0, std::pair<double, double> p1, std::pair<double, double> p2);

    /*!
     * Calculate the energy for moving on the straight line. The acceleration and deceleration times are encountered, but the
     * energy needed to perform the acceleration of deceleration is not encountered (as it should be
     * encountered in the calculate_turning_energy method)
     *
     * @param v_in The speed of the drone while entering the path segment
     * @param a_in Acceleration at the beginning of the segment
     * @param v_out The speed of the drone while leaving the path segment
     * @param a_out Acceleration (deceleration) at the end of the segment. Negative value always
     * @param p1 Start point of the path segment
     * @param p2 End point of the path segment
     * @return Energy consumption in Joules
     */
    [[nodiscard]] double calculate_straight_line_energy(double v_in, double a_in, double v_out, double a_out,
                                                        const std::pair<double, double> &p1,
                                                        const std::pair<double, double> &p2) const;

    /*!
     * Calculate the energy of a UAV moving along a straight line
     * @param v_in Initial speed along the movement direction [m/s]
     * @param a_in Acceleration along the movement direction [m/s^2]
     * @param v_out Output speed along the movement direction [m/s]
     * @param a_out Output acceleration (negative) along the movement direction m[s^2]
     * @param s length of the segment [m]
     * @return Energy consumption in Joules
     */
    [[nodiscard]] double
    calculate_straight_line_energy(double v_in, double a_in, double v_out, double a_out, double s) const;


    /*!
     * Calculate the energy spent by UAV when moving along a straight line between turns with such angles
     * @param turn1 Turn before the straight line segment
     * @param turn2 Turn after the straight line segment
     * @param s_tot Total distance between two turns
     * @return Energy consumption in Joules
     */
    [[nodiscard]] double calculate_straight_line_energy_between_turns(const turning_properties_t &turn1,
                                                                      const turning_properties_t &turn2,
                                                                      double s_tot) const;

    /*!
   * Get the energy spent on traversing a short lin, on which the optimal speed cannot be reached
   * @param v_in Speed of the UAV whole entering the segment
   * @param a_in Acceleration of the UAV
   * @param v_out Speed of the UAV wile leaving the segment
   * @param a_out Deceleration of UAV (negative value)
   * @param s Distance of the segment
   * @return Amount of energy spent to accelerate, and drop speed back to v_out moving only s meters
   */
    [[nodiscard]] double
    calculate_short_line_energy(double v_in, double a_in, double v_out, double a_out, double s) const;

    explicit EnergyCalculator(const energy_calculator_config_t &energy_calculator_config): EnergyCalculator(energy_calculator_config, std::make_shared<loggers::SimpleLogger>()) {};

    EnergyCalculator(const energy_calculator_config_t &energy_calculator_config, std::shared_ptr<loggers::SimpleLogger> logger);

    EnergyCalculator() = delete;

    /*!
     * Calculate the total energy spent to follow the path
     * NOTE: the calculation relies on the fact that the closest distance between two path points will allow the drone to
     * fully accelerate to the optimal speed, and decelerate from the optimal speed to 0
     *
     * @param path Path for which the total power consumption will be calculated
     * @return The amount of spent energy to follow the whole path in Joules [J]
     */
    [[nodiscard]] double calculate_path_energy_consumption(const std::vector<std::pair<double, double>> &path) const;

    /*!
     * @return average acceleration from the config
     */
    [[nodiscard]] double get_average_acceleration() const {
        return config.average_acceleration;
    }

    /*!
     * @return pre-calculated optimal speed of the UAV
     */
    [[nodiscard]] double get_optimal_speed() const { return v_r; }


    /*!
     * Calculate the energy spent to accelerate UAV from velocity v_in tp v_out in time time
     * @param v_in Starting velocity
     * @param v_out End velocity
     * @param time Time of acceleration
     * @return Energy spent
     */
    [[nodiscard]] double calculate_acceleration_energy(double v_in, double v_out, double time) const;


    void set_logger(std::shared_ptr<loggers::SimpleLogger> new_logger) {
        m_logger = std::move(new_logger);
    }

    // -----------------------------------------------------------
    // Set of functions for path time calculation

    /*!
     * Reset the internal counter for path times
     */
    void reset_path_time() { m_total_path_time = 0; };

    /*!
     * Get the travel time of all calculated paths after the constructor or reset_path_time() calls
     * @return total time in seconds
     */
    [[nodiscard]] double get_total_path_time() const { return m_total_path_time; };

    /*!
     * @return Power consumption on hover in Watts
     */
    [[nodiscard]] double get_hover_power() const { return P_h; };

    /*!
     * @return Power consumption when moving with the optimal speed [W]
     */
    [[nodiscard]] double get_optimal_speed_power() const { return P_r; };

};

#endif
