#ifndef MAP_TO_GRAPH_ALGORITHMS_HPP
#define MAP_TO_GRAPH_ALGORITHMS_HPP

#include <vector>
#include "MapPolygon.hpp"

using vpdd = std::vector<point_t>;

struct polygon_decomposition_error: public std::runtime_error {
    using runtime_error::runtime_error;
};

/*!
 * Calculate the sweeping path to cover the graph g
 * NOTE: If the area to cover is not a convex polygon, the algorithm will produce not a full coverage path
 * @param polygon: Polygon with only fly-zone that should be swept
 * @param angle: angle of sweeping lines
 * @param sweeping_step distance between sweeping lines
 * @param wall_distance how far from the top and bottom edge to make a turn. Can be negative to leave the fly zone
 * @param start_up Whether the first column should be covered by movement up or down
 * @return A sequence of points (longitude, latitude) that need to be visited by drone
 */
vpdd sweeping(const MapPolygon &polygon, double angle, double sweeping_step, double wall_distance=0, bool start_up=false);

/*!
 * Decompose the MapPolygon into smaller MapPolygons using trapezoidal decomposition algorithm
 * @param polygon MapPolygon to be decomposed
 * @param merge_to_boustrophedon If true, trapezoids will be merged if they have a common edge (boustrophedon decomposition)
 * @return Vector of convex polygons with no no-fly-zones and such that
 * union(result) = original_polygon
 * intersection(result) = {}
 */
std::vector<MapPolygon> trapezoidal_decomposition(const MapPolygon &polygon,  decomposition_type_t decomposition_type=BOUSTROPHEDON_DECOMPOSITION);

/*!
 * Create a map polygon from 2 segments and a vertical line on the right
 * @note Each vertical line must go through each fo 2 segments. Otherwise the result is unfeasible
 * @param s1 Segment 1
 * @param s2 Segment 2
 * @param x x coordinate of vertical line
 * @return MapPolygon created
 */
MapPolygon polygon_from_2_segments(segment_t s1, segment_t s2, double x);


/*!
 * Cover thin polygon, width of which is smaller than sweeping_step / 2 with at least one rotation angle
 * @param polygon Polygon to be covered
 * @param sweeping_step Sweeping step of the algorithm
 * @param wall_distance Distance to polygon walls while performing sweeping
 * @return Coverage paths if the polygon is truly that thin. Empty vector if the polygon is not thin enough
 */
vpdd thin_polygon_coverage(const MapPolygon &polygon, double sweeping_step, double wall_distance);


#endif //MAP_TO_GRAPH_ALGORITHMS_HPP
