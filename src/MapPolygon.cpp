#include "MapPolygon.hpp"
#include <iostream>
#include "utils.hpp"
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include "algorithms.hpp"

// TODO: make most of methods external functions (maybe, working on not MapPolygons but just on vector<pair<double, double>>
// TODO: make use of some external polygons library like
namespace {
    const double SPLIT_BIN_SEARCH_THRESH = 1;

    double get_mean_x(const polygon_t &pol) {
        double res = pol[0].first;
        for (size_t i = 1; i < pol.size(); ++i) {
            res += pol[i].first;
        }
        return res / pol.size();
    }

    std::pair<double, double> get_leftmost_rightmost(const vpdd &pol) {
        double rightmost_x = std::numeric_limits<double>::lowest(), leftmost_x = std::numeric_limits<double>::max();
        for (const auto &p: pol) {
            rightmost_x = std::max(rightmost_x, p.first);
            leftmost_x = std::min(leftmost_x, p.first);
        }
        return {leftmost_x, rightmost_x};
    }
}

MapPolygon::MapPolygon(const MapPolygon &p) {
    fly_zone_polygon_points = p.fly_zone_polygon_points;
    no_fly_zone_polygons = p.no_fly_zone_polygons;
}

MapPolygon MapPolygon::rotated(double angle) const {
    // Make a copy of this mapPolygon
    MapPolygon new_polygon;


    // Rotate each point of the polygon describing the fly-zone
    std::transform(fly_zone_polygon_points.begin(),
                   fly_zone_polygon_points.end(),
                   std::inserter(new_polygon.fly_zone_polygon_points, new_polygon.fly_zone_polygon_points.begin()),
                   ([angle](const auto &p) { return rotate_point(p, angle); }));


    // Rotate each point in each of the polygons describing non-fly zone
    std::transform(no_fly_zone_polygons.begin(),
                   no_fly_zone_polygons.end(),
                   std::inserter(new_polygon.no_fly_zone_polygons, new_polygon.no_fly_zone_polygons.begin()),
                   [angle](const auto &polygon) {
                       polygon_t no_fly_new_polygon;
                       std::transform(polygon.begin(),
                                      polygon.end(),
                                      std::inserter(no_fly_new_polygon, no_fly_new_polygon.begin()),
                                      [angle](const auto &p) { return rotate_point(p, angle); });
                       return no_fly_new_polygon;
                   });
    return new_polygon;
}


std::set<point_t> MapPolygon::get_all_points() const {
    std::set<point_t> points;
    std::copy(fly_zone_polygon_points.begin(), fly_zone_polygon_points.end(),
              std::inserter(points, points.begin()));
    std::for_each(no_fly_zone_polygons.begin(), no_fly_zone_polygons.end(),
                  [&](const auto &p) {
                      std::copy(p.begin(), p.end(), std::inserter(points, points.begin()));
                  });
    return points;
}

namespace {
    template<class T>
    bool find_point_neighbours(point_t point, std::pair<point_t, point_t> &res, const T &container) {
        if (container.size() < 3) {
            throw wrong_polygon_format_error("Polygon has less than 3 points");
        }
        if (container[0] == point) {
            res = {container[container.size() - 2], container[1]};
            return true;
        }

        // First and last points are always the same, so skip the last one for simplicity
        for (size_t i = 1; i < container.size() - 1; ++i) {
            if (container[i] == point) {
                res = {container[i - 1], container[i + 1]};
                return true;
            }
        }
        return false;
    }
}


std::pair<point_t, point_t> MapPolygon::point_neighbors(point_t point) const {
    std::pair<point_t, point_t> res;
    if (find_point_neighbours(point, res, fly_zone_polygon_points)) {
        return res;
    }
    for (auto &p: no_fly_zone_polygons) {
        if (find_point_neighbours(point, res, p)) {
            return res;
        }
    }
    throw non_existing_point_error("Point is not in the polygon");
}

[[nodiscard]] std::pair<point_t, point_t> MapPolygon::rightmost_edge() const {
    double rightmost_x = fly_zone_polygon_points[0].first;
    for (const auto &point: fly_zone_polygon_points) {
        rightmost_x = std::max(rightmost_x, point.first);
    }
    for (size_t i = 0; i < fly_zone_polygon_points.size() - 1; ++i) {
        if (fly_zone_polygon_points[i].first == rightmost_x) {
            return {fly_zone_polygon_points[i], fly_zone_polygon_points[i + 1]};
        }
    }
    throw wrong_polygon_format_error("Cannot find a rightmost point in polygon. Probably it is empty");
}

std::vector<segment_t> MapPolygon::get_all_segments() const {
    std::vector<segment_t> segments;
    for (size_t i = 0; i + 1 < fly_zone_polygon_points.size(); ++i) {
        segments.emplace_back(fly_zone_polygon_points[i], fly_zone_polygon_points[i + 1]);
    }
    for (const auto &pol: no_fly_zone_polygons) {
        for (size_t i = 0; i + 1 < pol.size(); ++i) {
            segments.emplace_back(pol[i], pol[i + 1]);
        }
    }
    return segments;
}

std::vector<double> MapPolygon::get_n_longest_edges_rotation_angles(size_t n) const {
    auto edges = get_all_segments();
    std::sort(edges.begin(), edges.end(), [](const segment_t &s1, const segment_t &s2) {
        return segment_length(s1) >
               segment_length(s2);
    });
    std::vector<double> rotations;
    for (size_t i = 0; i < n && i < edges.size(); ++i) {
        double segment_rotation = M_PI_2 - get_segment_rotation(edges[i]);
        rotations.push_back(segment_rotation);
        if (segment_rotation < M_PI) {
            rotations.push_back(segment_rotation + M_PI);
        } else {
            rotations.push_back(segment_rotation - M_PI);
        }
    }
    return rotations;
}


double MapPolygon::area() const {
    double whole_area = 0;
    for (size_t i = 1; i < fly_zone_polygon_points.size(); ++i) {
        whole_area += (fly_zone_polygon_points[i - 1].first + fly_zone_polygon_points[i].first) *
                      (fly_zone_polygon_points[i - 1].second - fly_zone_polygon_points[i].second);
    }
    return std::abs(whole_area / 2);
}

void MapPolygon::make_pure_convex() {
    if (fly_zone_polygon_points.empty()) {
        return;
    }
    std::vector<point_t> new_fly_zone{fly_zone_polygon_points[0]};
    for (size_t i = 1; i + 1 < fly_zone_polygon_points.size(); ++i) {
        if (std::abs(angle_between_vectors(fly_zone_polygon_points[i - 1], fly_zone_polygon_points[i],
                                           fly_zone_polygon_points[i + 1]) - M_PI) < 1e-4) {
            continue;
        }
        new_fly_zone.push_back(fly_zone_polygon_points[i]);
    }
    new_fly_zone.push_back(fly_zone_polygon_points.back());
    fly_zone_polygon_points = new_fly_zone;
}


std::pair<MapPolygon, MapPolygon> MapPolygon::split_by_vertical_line(double x) {
//    make_pure_convex();
    make_polygon_clockwise(fly_zone_polygon_points);
    auto fly_zone_copy = fly_zone_polygon_points;
    std::vector<std::pair<double, double>> new_pol[2];
    double rightmost_x[2] = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()};


    // Iterate through each segment, detecting crossing of the vertical line
    for (size_t i = 0; i + 1 < fly_zone_copy.size(); ++i) {
        if (fly_zone_copy[i].first == x) {
            new_pol[0].push_back(fly_zone_copy[i]);
            new_pol[1].push_back(fly_zone_copy[i]);
            continue;
        }
        if (fly_zone_copy[i].first < x) {
            new_pol[0].push_back(fly_zone_copy[i]);
        } else {
            new_pol[1].push_back(fly_zone_copy[i]);
        }

        if (fly_zone_copy[i + 1].first != x &&
            ((fly_zone_copy[i].first < x && fly_zone_copy[i + 1].first > x) ||
             (fly_zone_copy[i].first > x && fly_zone_copy[i + 1].first < x))) {
            auto intersection = segment_vertical_line_intersection({fly_zone_copy[i], fly_zone_copy[i + 1]}, x);
            new_pol[0].push_back(intersection);
            new_pol[1].push_back(intersection);
        }
    }
//    new_pol[cur_pol].push_back(fly_zone_copy.back());
    // Add starting point to make the polygon closed
    new_pol[0].push_back(new_pol[0].front());
    new_pol[1].push_back(new_pol[1].front());

    std::pair<MapPolygon, MapPolygon> res;

    for (int i = 0; i < 2; ++i)
        for (const auto &p: new_pol[i]) {
            rightmost_x[i] = std::max(rightmost_x[i], p.first);
        };

    // Place the polygon on the left to the left
    if (get_mean_x(new_pol[0]) < get_mean_x(new_pol[1])) {
        res.first.fly_zone_polygon_points = new_pol[0];
        res.second.fly_zone_polygon_points = new_pol[1];
    } else {
        res.first.fly_zone_polygon_points = new_pol[1];
        res.second.fly_zone_polygon_points = new_pol[0];
    }
    return res;
}


std::vector<MapPolygon> MapPolygon::split_into_pieces(double max_piece_area) {
    if (max_piece_area > area()) {
        return {*this};
    }
    make_polygon_clockwise(fly_zone_polygon_points);

    try {

        int res_polygons = std::ceil(area() / max_piece_area);
        double split_piece_area = area() / res_polygons;
        auto cur_polygon = *this;
        std::vector<MapPolygon> res;


        for (int i = 0; i < res_polygons - 1; ++i) {
            double leftmost_x, rightmost_x;
            std::tie(leftmost_x, rightmost_x) = get_leftmost_rightmost(cur_polygon.fly_zone_polygon_points);
            // Using bin-search find the appropriate vertical line
            double l = leftmost_x, r = rightmost_x;
            while (r - l > SPLIT_BIN_SEARCH_THRESH) {
                double m = (r + l) / 2;
                auto line_split = cur_polygon.split_by_vertical_line(m);
                if (line_split.first.area() < split_piece_area) {
                    l = m;
                } else {
                    r = m;
                }
            }
            // Take a piece from the left and continue the algorithm
            auto best_split = cur_polygon.split_by_vertical_line(l);
            res.push_back(best_split.first);
            cur_polygon = best_split.second;
        }
        res.push_back(cur_polygon);
        return res;
    } catch (std::runtime_error &e) {
//        m_logger->log_err("[PathGenerator]: ERROR while dividing polygon: " + std::string(e.what()));
    }
    return {};
}

point_t MapPolygon::leftmost_point() const {
    return std::reduce(fly_zone_polygon_points.begin(), fly_zone_polygon_points.end(),
                       std::make_pair(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
                       [](const auto &p1, const auto &p2) { return p1.first < p2.first ? p1 : p2; });
}

point_t MapPolygon::rightmost_point() const {
    return std::reduce(fly_zone_polygon_points.begin(), fly_zone_polygon_points.end(),
                       std::make_pair(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()),
                       [](const auto &p1, const auto &p2) { return p1.first < p2.first ? p2 : p1; });
}


std::optional<double> MapPolygon::is_thinner_than_rotation(double width) const {
    for (const auto &segment: get_all_segments()) {
        auto segment_rotation = get_segment_rotation(segment);
        segment_rotation += segment_rotation < M_PI ? M_PI_2 : -M_PI_2;
        auto rotated_polygon = rotated(segment_rotation);

        if ((rotated_polygon.rightmost_point().first - rotated_polygon.leftmost_point().first) < width) {
            std::cout << "Thin polygon found" << std::endl;
            return segment_rotation;
        }
    }
    return std::nullopt;
}

double MapPolygon::height() const {
    double highest_y = std::numeric_limits<double>::lowest();
    double lowest_y = std::numeric_limits<double>::max();

    // Assuming that no no-fly zone is located outside the fly-zone
    for (const auto &p: fly_zone_polygon_points) {
        highest_y = std::max(highest_y, p.second);
        lowest_y = std::min(lowest_y, p.second);
    }
    return highest_y - lowest_y;
}


namespace {
    std::optional<double> max_piece_area_for_number(std::vector<MapPolygon> &sub_polygons, size_t n) {
        if (n <= sub_polygons.size()) {
            return std::nullopt;
        }

        // Using the binary search, find the max piece area, corresponding to the optimal decomposition
        double largest_area = 0;
        for (const auto &pol: sub_polygons) {
            largest_area = std::max(largest_area, pol.area());
        }
        double l = 0, r = largest_area;
        auto current_num_of_pols = sub_polygons.size();
        while (r - l > 1 and current_num_of_pols != n) {
            double m = (r + l) / 2;
            size_t number_of_decomposed = 0;
            for (auto &pol: sub_polygons) {
                number_of_decomposed += pol.split_into_pieces(m).size();
            }
            current_num_of_pols = number_of_decomposed;
            if (number_of_decomposed > n) {
                l = m;
            } else if (number_of_decomposed < n) {
                r = m;
            }
        }
        return (r + l) / 2;
    }
}

std::vector<MapPolygon> split_into_number(std::vector<MapPolygon> &sub_polygons, size_t n) {
    if (n <= sub_polygons.size()) {
        return sub_polygons;
    }

    // TODO: check if a better algorithm is needed here
    auto area = max_piece_area_for_number(sub_polygons, n);
    if (!area.has_value()) {
        return sub_polygons;
    }

    std::vector<MapPolygon> res;
    for (auto &p: sub_polygons) {
        auto decomposed = p.split_into_pieces(area.value());
        res.insert(res.end(), decomposed.begin(), decomposed.end());
    }
    return res;
}

namespace {
    double get_decomposition_cost(const std::vector<MapPolygon> &decomposition) {
        double res = 0;
        for (const auto &pol: decomposition) {
            res += pol.height();
        }
        return res;
    }
}


std::vector<double>
n_best_init_decomp_angles(const MapPolygon &m, int n, decomposition_type_t decomposition_type, double angle_eps) {
    std::vector<std::pair<double, double>> angles_costs;

    // Iterate through the angle of each polygon edge and try to find the best one
    for (auto &e: m.get_all_segments()) {
        auto rotation = -get_segment_rotation(e);
        auto decomp = trapezoidal_decomposition(m.rotated(rotation - M_PI_2 + 0.01), decomposition_type);
        angles_costs.emplace_back(get_decomposition_cost(decomp), rotation - M_PI_2 + 0.01);
    }

    std::sort(angles_costs.begin(), angles_costs.end());
    std::vector<double> res;

    for (size_t i = 0; i < std::min(static_cast<size_t>(n), angles_costs.size()); ++i) {
        bool is_close = false;
        for (const auto &angle: res) {
            if (std::abs(angle - angles_costs[i].second) < angle_eps) {
                is_close = true;
                break;
            }
        }
        if (is_close) {
            ++n;
            continue;
        }
        res.push_back(angles_costs[i].second);
    }
    return res;
}