#include "ShortestPathCalculator.hpp"
#include <algorithm>
#include "utils.hpp"
#include <cmath>

namespace {
    const double EPS = 1e-5;

    point_t pm(point_t point) {
        return {point.first * 1000, point.second * 1000};
    }

    /*!
     * Calculate the sets of polygon point pairs for which there is no direct path in a straight line.
     *
     * The set will include the pairs that can "see" each other through the center of the polygon for no-fly-zones
     * e.g. all possible pairs for a convex polygon except of neighbouring ones
     *
     * For a fly-zone, the set will include points that can "see" each other from outside of the polygon
     * e.g. if the polygon is concave, there is always a pair of points that can "see" each other without
     * any segments between them, but the direct path goes outside of the fly-zone
     *
     * @param polygon Polygon as a set of vertices
     * @param fly_zone Whether the polygon should be treated as a fly-zone
     * @return Set of polygon vertices between which there is no direct path not leaving the fly-zone
     */
    std::set<segment_t> no_direct_path(const std::vector<point_t> &polygon, bool fly_zone = true) {

        // Vector of turning angles around each polygon node
        std::vector<double> angles;
        angles.push_back(angle_between_vectors(pm(polygon[polygon.size() - 2]), pm(polygon[0]), pm(polygon[1])));
        for (size_t i = 1; i + 1 < polygon.size(); ++i) {
            angles.push_back(angle_between_vectors(pm(polygon[i - 1]), pm(polygon[i]), pm(polygon[i + 1])));
        }


        std::set<segment_t> no_direct_view;
        // Iterate through each pair of points in the polygon and if the sum of angles between them is not suitable -
        // add them to the list of not possible ones
        for (size_t i = 0; i < angles.size(); ++i) {
            double angle = 0;
            for (size_t j = i + 2; j < angles.size(); j++) {
                angle += M_PI - angles[j - 1];
                if (((angle < 0 - EPS || angle > 2 * M_PI + EPS) && fly_zone) ||
                    ((angle < 2 * M_PI - EPS && angle > 0 + EPS) && !fly_zone)) {
                    no_direct_view.insert({polygon[i], polygon[j]});
                }
            }
        }
        // If first and last points were added accidentally, remove them
        auto pos = no_direct_view.find({polygon.front(), polygon[polygon.size() - 2]});
        if (pos != no_direct_view.end()) {
            no_direct_view.erase(pos);
        }
        return no_direct_view;
    }

    /*!
     * Calculate the segments, between which there is no direct path definitely.
     * Needed for the main algorithm while checking if there is a direct path by
     * "visibility" of the points (no segment between them). But this situation can also
     * happen when nodes "see" each other through a no-fly zone (e.g. different points of a convex no-fly zone
     * of when the fly-zone if non-convex, so there always exist a line between
     * @param polygon Polygon for which segments will be found
     * @return
     */
    std::set<segment_t> no_direct_path(const MapPolygon &polygon) {
        std::set<segment_t> no_direct_view = no_direct_path(polygon.fly_zone_polygon_points, true);
        for (const auto &p: polygon.no_fly_zone_polygons) {
            auto no_fly_zone_no_view = no_direct_path(p, false);
            no_direct_view.insert(no_fly_zone_no_view.begin(), no_fly_zone_no_view.end());
        }
        return no_direct_view;
    }
}

ShortestPathCalculator::ShortestPathCalculator(const MapPolygon &polygon) {
    auto points_tmp = polygon.get_all_points();
    m_polygon_points = std::vector<point_t>{points_tmp.begin(), points_tmp.end()};
    m_polygon_segments = polygon.get_all_segments();

    // Assign each point a unique identifier to be able to quickly traverse through it
    int index = 0;
    for (const auto &p: m_polygon_points) {
        m_point_index[p] = index++;
    }

    // Create a 2-d matrix that will contain paths for the shortest path between each of perimeter point
    // And 2-d matrix for the Floyd-Warshall algorithm
    m_next_vertex_in_path = std::vector<std::vector<size_t>>(m_polygon_points.size());
    std::for_each(m_next_vertex_in_path.begin(), m_next_vertex_in_path.end(),
                  [&](auto &row) { row = std::vector<size_t>(m_polygon_points.size(), -1); });
    m_floyd_warshall_d = std::vector<std::vector<double>>(m_polygon_points.size());
    std::for_each(m_floyd_warshall_d.begin(), m_floyd_warshall_d.end(),
                  [&](auto &row) { row = std::vector<double>(m_polygon_points.size(), HUGE_VAL); });

    // Get the list of pairs of points, for which there is no direct path even if they can "see" each other
    auto no_direct_path_pairs = no_direct_path(polygon);

    // O(N^3) building of the matrix. Ok as the algorithm itself runs on O(N^3)
    for (size_t i = 0; i < m_polygon_points.size(); i++) {
        m_floyd_warshall_d[i][i] = 0;
        for (size_t j = i + 1; j < m_polygon_points.size(); j++) {
            segment_t segment_between_point = {m_polygon_points[i], m_polygon_points[j]};
            if (m_polygon_points[i] == m_polygon_points[j]) {
                continue;
            }

            // If there is no direct path by previously calculated algorithm, omit this pair
            if (no_direct_path_pairs.find(segment_between_point) != no_direct_path_pairs.end() ||
                no_direct_path_pairs.find({segment_between_point.second, segment_between_point.first}) !=
                no_direct_path_pairs.end()) {
                continue;
            }

            bool segment_intersects = false;
            for (const auto &segment: m_polygon_segments) {
                // If there is an exact edge between these two points -- break as there won't be any intersections anymore
                if ((segment.first == segment_between_point.first && segment.second == segment_between_point.second) ||
                    (segment.second == segment_between_point.first && segment.first == segment_between_point.second)) {
                    segment_intersects = false;
                    break;
                }

                // If the segment starts in point - don't check it for intersection as it will intersect in that point
                if (segment.first == segment_between_point.first || segment.second == segment_between_point.first ||
                    segment.first == segment_between_point.second || segment.second == segment_between_point.second) {
                    continue;
                }

                if (segments_intersect(segment, segment_between_point)) {
                    segment_intersects = true;
                    break;
                }
            }
            if (!segment_intersects) {
                // Update Floyd Warshall algorithm matrix
                m_floyd_warshall_d[i][j] = m_floyd_warshall_d[j][i] = segment_length(segment_between_point);
                m_next_vertex_in_path[i][j] = j;
                m_next_vertex_in_path[j][i] = i;
            }
        }
    }
    run_floyd_warshall();
}


void ShortestPathCalculator::run_floyd_warshall() {
    const size_t N = m_floyd_warshall_d.size();
    for (size_t k = 0; k < N; ++k) {
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                if (m_floyd_warshall_d[i][j] > m_floyd_warshall_d[i][k] + m_floyd_warshall_d[k][j]) {
                    m_floyd_warshall_d[i][j] = m_floyd_warshall_d[i][k] + m_floyd_warshall_d[k][j];
                    m_floyd_warshall_d[j][i] = m_floyd_warshall_d[i][j];
                    // Now, to get to j from i, we should in direction to k (to next vertex in path to k)
                    m_next_vertex_in_path[i][j] = m_next_vertex_in_path[i][k];
                    m_next_vertex_in_path[j][i] = m_next_vertex_in_path[j][k];
                }
            }
        }
    }
}

std::vector<point_t> ShortestPathCalculator::get_approximate_shortest_path(point_t p1, point_t p2) const {
    if (point_can_see_point(p1, p2)) {
        return std::vector<point_t>{p1, p2};
    }

    point_t closest_to_start = closest_polygon_point(p1);
    point_t closest_to_end = closest_polygon_point(p2);

    auto path_between_closest = shortest_path_between_polygon_nodes(m_point_index[closest_to_start],
                                                                    m_point_index[closest_to_end]);
    // TODO: can check if the second path point can be reached directly from p1 to make path feasible

    path_between_closest.insert(path_between_closest.begin(), p1);
    path_between_closest.push_back(p2);
    return path_between_closest;

}

point_t ShortestPathCalculator::closest_polygon_point(point_t p) const {
    if (m_polygon_points.empty()) {
        throw shortest_path_calculation_error("No point in the polygon found");
    }
    point_t closest_point = m_point_index.begin()->first;
    double closest_distance = distance_between_points(closest_point, p);
    for (const auto &point: m_polygon_points) {
        double new_distance = distance_between_points(point, p);
        if (new_distance < closest_distance) {
            closest_distance = new_distance;
            closest_point = point;
        }
    }
    return closest_point;
}


std::vector<point_t> ShortestPathCalculator::shortest_path_between_polygon_nodes(size_t i, size_t j) const {
    // Use the matrix, built using Floyd Warshall to traverse through the shortest path
    std::vector<point_t> res;
    while (i != j) {
        res.push_back(m_polygon_points[i]);
        i = m_next_vertex_in_path[i][j];
    }
    res.push_back(m_polygon_points[j]);
    return res;
}

std::vector<point_t> ShortestPathCalculator::shortest_path_between_points(point_t p1, point_t p2) const {
    // If path is saved to cache - return it from there
    auto path_in_cache = paths_cache.find({p1, p2});
    if (path_in_cache != paths_cache.end()) {
        return path_in_cache->second;
    }
    if (point_can_see_point(p1, p2)) {
        return {p1, p2};
    }

    std::vector<size_t> seen_from_p1, seen_from_p2;
    for (const auto &p: m_polygon_points) {
        if (point_can_see_point(p1, p)) {
            seen_from_p1.push_back(m_point_index[p]);
        }
        if (point_can_see_point(p2, p)) {
            seen_from_p2.push_back(m_point_index[p]);
        }
    }
    if (seen_from_p2.empty() || seen_from_p1.empty()) {
        // Should never get here. Just return a value to not throw exceptions
        return {p1, p2};
    }

    size_t best_i_neighbor = seen_from_p1[0];
    size_t best_j_neighbor = seen_from_p2[0];
    double best_path_cost = std::numeric_limits<double>::max();
    for (auto from_p1: seen_from_p1) {
        for (auto from_p2: seen_from_p2) {
            double path_cost = distance_between_points(p1, m_polygon_points[from_p1]) +
                               distance_between_points(p2, m_polygon_points[from_p2]) +
                               m_floyd_warshall_d[from_p1][from_p2];
            if (path_cost < best_path_cost) {
                best_path_cost = path_cost;
                best_i_neighbor = from_p1;
                best_j_neighbor = from_p2;
            }
        }
    }
    auto res = shortest_path_between_polygon_nodes(best_i_neighbor, best_j_neighbor);
    res.insert(res.begin(), p1);
    res.insert(res.end(), p2);
    paths_cache[{p1, p2}] = res;
    return res;
}

size_t
ShortestPathCalculator::farthest_point_seen_in_path(point_t source_point, const std::vector<point_t> &path) const {
    size_t res = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        if (point_can_see_point(source_point, path[i])) {
            res = i;
        }
    }
    return res;
}

bool ShortestPathCalculator::point_can_see_point(point_t p1, point_t p2) const {
    segment_t segment{p1, p2};
    for (const auto &border_segment: m_polygon_segments) {
        if (segments_intersect(segment, border_segment)) {
            return false;
        }
    }
    return true;
}

