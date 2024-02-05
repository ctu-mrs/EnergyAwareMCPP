#include "mstsp_solver/TargetSet.h"
#include "algorithms.hpp"
#include <iostream>
#include <algorithm>
#include <utility>

namespace mstsp_solver {

    TargetSet::TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step, double wall_distance,
                         EnergyCalculator energy_calculator,
                         const std::vector<double> &rotation_angles) : index(index), polygon(polygon),
                                                                       energy_calculator(std::move(energy_calculator)),
                                                                       sweeping_step(sweeping_step),
                                                                       m_wall_distance{wall_distance} {
        set_rotation_angles(rotation_angles);
    }


    TargetSet::TargetSet(size_t index,
                         const MapPolygon &polygon,
                         double sweeping_step,
                         double wall_distance,
                         EnergyCalculator energy_calculator,
                         size_t number_of_edges_rotations) : index(index), polygon(polygon),
                                                             energy_calculator(std::move(energy_calculator)),
                                                             sweeping_step(sweeping_step),
                                                             m_wall_distance{wall_distance} {

        auto thin_coverage = thin_polygon_coverage(polygon, sweeping_step, 4);
        // If no thin coverage path is generated because the polygon is not thin enough, perform normal sweeping procedure
        if (thin_coverage.empty()) {
            set_rotation_angles(polygon.get_n_longest_edges_rotation_angles(number_of_edges_rotations));
        } else {
            targets.push_back(Target{
                    true, 0.0, 0.0, thin_coverage[0], thin_coverage.back(), index, targets.size()
            });
        }


    }

    void TargetSet::add_one_rotation_angle(double angle, bool up) {
        auto sweeping_path = sweeping(polygon, angle, sweeping_step, m_wall_distance, up);
        // If sweeping failed (e.g. because of the polygon splitting with such a rotation angle)
        if (sweeping_path.empty()) {
            return;
        }
        double path_energy = energy_calculator.calculate_path_energy_consumption(sweeping_path);

        targets.push_back(Target{up,
                                 angle,
                                 path_energy,
                                 sweeping_path[0],
                                 sweeping_path[sweeping_path.size() - 1],
                                 index,
                                 targets.size()});
    }


    void TargetSet::set_rotation_angles(const std::vector<double> &angles) {
        targets.clear();
        for (auto angle: angles) {
            for (int i = 0; i < 2; i++) {
                add_one_rotation_angle(angle, static_cast<bool>(i));
            }
        }
        if (targets.empty()) {
            // If not sweeping angle produced a valid sweeping pattern
            // Try to add the sweeping with no angle. This should work for any polygon after boustrophedon decomposition
            add_one_rotation_angle(0, true);
            add_one_rotation_angle(0, false);
        }
    }
}