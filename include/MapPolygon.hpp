#ifndef MAP_TO_GRAPH_MAPPOLYGON_HPP
#define MAP_TO_GRAPH_MAPPOLYGON_HPP

#include <vector>
#include <string>
#include <array>
#include <stdexcept>
#include <numeric>
#include <set>
#include <map>
#include <algorithm>
#include "custom_types.hpp"
#include "utils.hpp"
#include <memory>
#include <SimpleLogger.h>
#include <optional>


struct non_existing_point_error : public std::runtime_error {
    using runtime_error::runtime_error;
};

struct wrong_polygon_format_error : public std::runtime_error {
    using runtime_error::runtime_error;
};


/*!
 * Structure for representation of a polygon with holes
 */
struct MapPolygon {
public:
    using polygon_t = std::vector<point_t>;

    // TODO: consider making these fields private and accessible only through getters
    polygon_t fly_zone_polygon_points;
    std::vector<polygon_t> no_fly_zone_polygons;

    MapPolygon() = default;

    MapPolygon(const MapPolygon &p);

    /*!
     * Constructor for creating a MapPolygon from something like GeneratePaths.srv ROS message. Accepts fly zone and
     * no-fly zones in {latitude, longitude} format as vectors and transforms internally into meters
     * @param fly_zone Description of a fly zone
     * @param no_fly_zones Vector of descriptions of each no fly zone
     * @param gps_transform_origin GPS coordinates of the point, that will be mapped to (0, 0) after transformation to meters
     */
    MapPolygon(std::vector<point_t> &fly_zone, const std::vector<std::vector<point_t>> &no_fly_zones,
               std::pair<double, double> gps_transform_origin) {
        std::for_each(fly_zone.begin(), fly_zone.end(), [&](const auto &p) {
            fly_zone_polygon_points.push_back(gps_coordinates_to_meters({p.first, p.second}, gps_transform_origin));
        });
        make_polygon_clockwise(fly_zone_polygon_points);
        for (auto &no_fly_zone: no_fly_zones) {
            no_fly_zone_polygons.emplace_back();
            std::for_each(no_fly_zone.begin(), no_fly_zone.end(), [&](const auto &p) {
                no_fly_zone_polygons.back().push_back(
                        gps_coordinates_to_meters({p.first, p.second}, gps_transform_origin));
            });
            make_polygon_clockwise(no_fly_zone_polygons[no_fly_zone_polygons.size() - 1]);
        }
    }

    /*!
    * Constructor for creating a MapPolygon from known fly and no-fly zones
    * @param fly_zone Description of a fly zone
    * @param no_fly_zones Vector of descriptions of each no fly zone
    */
    MapPolygon(std::vector<point_t> &fly_zone, const std::vector<std::vector<point_t>> &no_fly_zones) {
        fly_zone_polygon_points = fly_zone;
        make_polygon_clockwise(fly_zone_polygon_points);
        no_fly_zone_polygons = no_fly_zones;
        for (auto &no_fly_zone: no_fly_zone_polygons) {
            make_polygon_clockwise(no_fly_zone);
        }
    }


    MapPolygon &operator=(const MapPolygon &rhs) = default;

    /*!
     * Get the list of all the polygons point (both fly-zone and non-fly zone)
     * @return Set of all the points
     */
    [[nodiscard]] std::set<point_t> get_all_points() const;

    /*!
     * Get the vector of all segments (of both fly and no-fly zones of the map polygon)
     * @return Vector of all the present segments
     */
    [[nodiscard]] std::vector<segment_t> get_all_segments() const;

    /*!
     * Get two neighbouring points in some polygon for the specified point
     * @param point point that belongs to one of polygons
     * @return pair of two points - neighbors of the points
     */
    [[nodiscard]] std::pair<point_t, point_t> point_neighbors(point_t point) const;

    /*!
     * Get the map polygon, rotated by angle radians counter clockwise around the coordinates origin
     * @param angle Angle of the rotation [rad]
     * @return New MapPolygon with all the points rotated by the angle
     */
    [[nodiscard]] MapPolygon rotated(double angle) const;

    /*!
     * Find the edge starting from the first rightmost point
     * @return Edge starting from the first rightmost point
     */
    [[nodiscard]] std::pair<point_t, point_t> rightmost_edge() const;

    /*!
     * Get n (or less if n >= number_of_edges) angles of rotation of n longest edges
     * @param n Number of longest edges to encounter
     * @return vector of min(n, number_of_edges) rotation angles
     */
    [[nodiscard]] std::vector<double> get_n_longest_edges_rotation_angles(size_t n) const;

    /*!
     * Split fly-zone by a vertical line. If there are non-fly zones, they will be just deleted
     * @param x X coordinate of the vertical line
     * @return Two polygons, left one is first, right one is second
     */
    std::pair<MapPolygon, MapPolygon> split_by_vertical_line(double x);

    /*!
     * Split the convex polygon with no no-fly zones into the smallest number of
     * equal area pieces
     * @warning Works properly only for convex polygons with no no-fly zones
     * @param max_piece_area Max area of one piece
     * @return vector of the result of decomposition
     */
    std::vector<MapPolygon> split_into_pieces(double max_piece_area);

    /*!
     * Make the fly-zone purely convex by replacing it with its convex hull
     */
    void make_pure_convex();

    /*!
     * @return area of the fly-zone
     */
    [[nodiscard]] double area() const;

    /*!
     * @return fly-zone point with the lowest X coordinate. Any of them if multiple exist
     */
    [[nodiscard]] point_t leftmost_point() const;

    /*!
     * @return fly-zone point with the largest X coordinate. Any of them if multiple exist
     */
    [[nodiscard]] point_t rightmost_point() const;


    // TODO: rename the function below
    /*!
     * @return the rotation that should be applied to po
     */
    [[nodiscard]] std::optional<double> is_thinner_than_rotation(double width) const;

    [[nodiscard]] double height() const;

};


/*!
 * Split polygons into the at least specified number of smaller ones
 * @param sub_polygons Initial sub polygons after decomposition
 * @param n Minimum number of polygons after the decomposition
 * @return Decomposed polygons
 */
std::vector<MapPolygon> split_into_number(std::vector<MapPolygon> &sub_polygons, size_t n);


/*!
 * Get at most n rotations that are assumed to be best
 * @param m Initial not decomposed polygon
 * @param n Maximum number of angled
 * @param decomposition_type type of decomposition used
 * @param angle_eps Epsilon up to which consider the angles to be the same (to prevent a couple of very similar ones)
 * @return
 */
std::vector<double>
n_best_init_decomp_angles(const MapPolygon &m, int n, decomposition_type_t decomposition_type, double angle_eps = 0.1);


#endif //MAP_TO_GRAPH_MAPPOLYGON_HPP
