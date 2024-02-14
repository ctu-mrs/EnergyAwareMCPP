//
// Created by mrs on 23.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_CUSTOM_TYPES_HPP
#define THESIS_TRAJECTORY_GENERATOR_CUSTOM_TYPES_HPP

#include <vector>

// Just convenient defines for some types
using point_t = std::pair<double, double>;
using segment_t = std::pair<point_t, point_t>;
using polygon_t = std::vector<std::pair<double, double>>;

#endif //THESIS_TRAJECTORY_GENERATOR_CUSTOM_TYPES_HPP
