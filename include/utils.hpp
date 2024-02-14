#ifndef MAP_TO_GRAPH_UTILS_HPP
#define MAP_TO_GRAPH_UTILS_HPP

#include <tuple>
#include <vector>
#include "custom_types.hpp"
#include <cmath>

using hom_t = std::tuple<double, double, double>;

hom_t cross_product(const hom_t &a, const hom_t &b);


enum decomposition_type_t {TRAPEZOIDAL_DECOMPOSITION,
    BOUSTROPHEDON_DECOMPOSITION,
    BOUSTROPHEDON_WITH_CONVEX_POLYGONS,

    DECOMPOSITION_TYPES_NUMBER // This should always be the last element as it's conversion to int represents the number of elements in this enum
};

/*!
 * Get the point of intersection of a segment and lone
 * @param p1 segment point 1
 * @param p2 segment point 2
 * @param line homography representation of the line
 * @return point of the intersection of the segment and the line
 */
std::pair<double, double> segment_line_intersection(const std::pair<double, double> &p1, 
                                                    const std::pair<double, double> &p2,
                                                    const hom_t &line);


/*!
 * Get the point, obtained by rotating the point p counter clockwise by angle
 * @param angle Angle of the counter clockwise point rotation
 * @param p Point to be rotated in the representation {x, y}
 * @return Points obtained by rotation
 */
point_t rotate_point(point_t p, double angle);

/*!
 * Get the intersection of 2 segments
 * @param s1 segment 1
 * @param s2 segment 2
 * @return intersection point
 */
point_t segment_segment_intersection(const segment_t &s1, const segment_t &s2);

/*!
 * @return length of the segment
 */
double segment_length(const segment_t &segment);

/*!
 * @param s1 segment 1
 * @param s2 segment 2
 * @return true if segments intersect, false otherwise
 */
bool segments_intersect(const segment_t &s1, const segment_t &s2);

/*!
 * @param p1 point 1
 * @param p2 point 2
 * @param p3 point 3
 * @return angle between segments (p1, p2) (p2, p3) in radians
 */
double angle_between_vectors(point_t p1, point_t p2, point_t p3);

/*!
 * @param p1 point 1
 * @param p2 point 2
 * @return euclidean distance between p1 and p2
 */
double distance_between_points(point_t p1, point_t p2);


/*!
 * @note this and mirrored functions work well only in small scale as the Earth is not flat
 * Convert gps coordinate to meters with respect to the origin point (origin will be transformed to (0, 0)
 * @param p gps coordinates point
 * @param origin origin in gps coordinates
 * @return point in meters from the origin
 */
point_t gps_coordinates_to_meters(point_t p, point_t origin);

/*!
 * Mirror function of @fn gps_coordinates_to_meter. Convert point in meters and origin to the initial gps coordinates
 * @param p point in meters
 * @param origin origin point in gps coordinates
 * @return gps coordinates of point p
 */
point_t meters_to_gps_coordinates(point_t p, point_t origin);

/*!
 * @return random number in range [0, max(int)]
 */
int generate_random_number();

/*!
 * @param polygon vector of points representing a polygon borders
 * @return whether the polygon is convex
 */
bool polygon_convex(std::vector<point_t> polygon);

/*!
 * Make polygon points be stored in clockwise order (when the out
 * @param polygon
 */
void make_polygon_clockwise(polygon_t &polygon);

/*!
 * Get the rotation of the segment (as vector starting at first point) according to the Ox axis
 * @param segment Segment, rotation for which will be found
 * @return Angle of the rotation in radians
 */
double get_segment_rotation(segment_t segment);

/*!
 * Check if two numbers are close enough
 * @param n1 Number 1
 * @param n2 Number 2
 * @param eps Possible error
 * @return true if the distance between n1 and n2 is less than EPS
 */
template<typename T>
bool isclose(T n1, T n2, double eps) {
    return std::abs(n1 - n2) < eps;
}

/*!
 * @param p1 First point
 * @param p2 Second point
 * @param eps Epsilon for each coordinate
 * @return true if two points are closer than eps in each coordinate
 */
inline bool isclose(point_t p1, point_t p2, double eps) {
    return isclose(p1.first, p2.first, eps) && isclose(p1.second, p2.second, eps);
}

/*!
 * Solve quadratic equation ax^2 + bx + c = 0
 * @return {FP_NAM, FP_NAN} if equation has no real roots, {r, r} if r is the only root,
 * {r1, r2} where r1 and r2 are the routes otherwise
 */
std::pair<double, double> solve_quadratic(double a, double b, double c);

/*!
 * Get the intersection of the segment and a vertical line
 * @param s segment
 * @param x line parameter of line equation X = x
 * @return point of intersection. If point does not belong to the segment, it is still returned
 */
point_t segment_vertical_line_intersection(const segment_t &s, double x);





#endif

