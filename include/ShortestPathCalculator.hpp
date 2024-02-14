#ifndef THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP
#define THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP

#include <map>
#include <vector>
#include "MapPolygon.hpp"
#include <unordered_map>

struct shortest_path_calculation_error: public std::runtime_error {
    using runtime_error::runtime_error;
};

/*!
 * Struct to make hashing of pair of points possible
 */
struct point_pair_hash {
    size_t operator()(const std::pair<point_t, point_t> &p) const {
        auto hash = std::hash<double>{};
        return hash(p.first.first) + hash(p.first.second) + hash(p.second.first) + hash(p.second.second);
    };
};

/*!
 * Class for calculation of the shortest path between points inside a polygon
 */
class ShortestPathCalculator {
private:
    std::vector<segment_t> m_polygon_segments;
    mutable std::map<point_t, int> m_point_index;
    std::vector<point_t> m_polygon_points;
    std::vector<std::vector<double>> m_floyd_warshall_d;
    std::vector<std::vector<size_t>> m_next_vertex_in_path;
    mutable std::unordered_map<std::pair<point_t, point_t>, std::vector<std::pair<double, double>>, point_pair_hash> paths_cache;

    /*!
     * Run the Floyd-Warshall on initial matrix to calculate shortest paths between all
     */
    void run_floyd_warshall();


    /*!
     * Find the shortest path between polygon nodes with index i and index j
     * @param i index of the first point
     * @param j index of the second point
     * @return path between those points including start and end
     */
    std::vector<point_t> shortest_path_between_polygon_nodes(size_t i, size_t j) const;

    /*!
     * Find the farthest point in path, seen from the source point
     * @param source_point Source point, from which "beams" will be shot
     * @param path Path consisting of points inside or on the edge of the polygon
     * @return Index of the last point seen in the path
     */
    size_t farthest_point_seen_in_path(point_t source_point, const std::vector<point_t> &path) const;

    bool point_can_see_point(point_t p1, point_t p2) const;


    /*!
     *  Find the closest point in polygon to yhe given point
     * @param p point to which the closest point will be found
     * @return The closest point to p
     */
    point_t closest_polygon_point(point_t p) const;

public:
    /*!
     * Main and the only constructor of the calculator.
     * @param polygon polygon, bounds of which will define the shortest path
     */
    explicit ShortestPathCalculator(const MapPolygon &polygon);

    ShortestPathCalculator() = delete;

    /*!
     * Method for getting the approximate shortest path between two points
     * @note It works fine only all two points are located close to nodes of the map polygon.
     * @note The path can even be unfeasible (going through a no-fly zone)
     * @param p1 Point to find path from
     * @param p2 Point to find path to
     * @return Path between teo points, where path[0] = p1, path[-1] = p2
     */
    std::vector<point_t> get_approximate_shortest_path(point_t p1, point_t p2) const;

    /*!
     * Get the exact Euclidean shortest path between two points inside of the polygon
     * @param p1 source point
     * @param p2 destination point
     * @return path between point including the start and end node
     */
    std::vector<point_t> shortest_path_between_points(point_t p1, point_t p2) const;

};


#endif //THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP
