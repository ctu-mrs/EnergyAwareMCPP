#include "utils.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <random>
//#include <mrs_lib/gps_conversions.h>
#include <algorithm>


const double METERS_IN_DEGREE = 111319.5;


hom_t cross_product(const hom_t &a, const hom_t &b) {
    return {std::get<1>(a) * std::get<2>(b) - std::get<2>(a) * std::get<1>(b),
            std::get<2>(a) * std::get<0>(b) - std::get<0>(a) * std::get<2>(b),
            std::get<0>(a) * std::get<1>(b) - std::get<1>(a) * std::get<0>(b)};
}

point_t segment_line_intersection(const point_t &p1,
                                                    const point_t &p2,
                                                    const hom_t &line) {
    hom_t p1_h = {p1.first, p1.second, 1};
    hom_t p2_h = {p2.first, p2.second, 1};
    hom_t segment_line = cross_product(p1_h, p2_h);
    hom_t p_intersection_h = cross_product(segment_line, line);
    return {std::get<0>(p_intersection_h) / std::get<2>(p_intersection_h),
            std::get<1>(p_intersection_h) / std::get<2>(p_intersection_h)};
}



std::pair<double, double> rotate_point(std::pair<double, double> p, double angle) {
    return {p.first * std::cos(angle) - p.second * std::sin(angle),
            p.second * std::cos(angle) + p.first * std::sin(angle)};
}


std::pair<double, double> segment_segment_intersection(const segment_t &s1, const segment_t &s2) {
    hom_t p1 = {s2.first.first, s2.first.second, 1};
    hom_t p2 = {s2.second.first, s2.second.second, 1};
    hom_t line = cross_product(p1, p2);
    return segment_line_intersection(s1.first, s1.second, line);
}

double segment_length(const segment_t &segment) {
    return std::sqrt(std::pow(segment.second.first - segment.first.first, 2) +
                     std::pow(segment.second.second - segment.first.second, 2));
}


bool segments_intersect(const segment_t &s1, const segment_t &s2) {
    // Consider that segments that have a common point does not intersect
    if (isclose(s1.first, s2.first, 1e-5) || isclose(s1.first, s2.second, 1e-5) ||
    isclose(s1.second, s2.first, 1e-5) || isclose(s1.second, s2.second, 1e-5)) {
        return false;
    }

    auto intersection = segment_segment_intersection(s1, s2);
    // Return true only of the intersection of lines is between the constraints of the first and the second segments
    return intersection.first >= std::min(s1.first.first, s1.second.first) &&
           intersection.first <= std::max(s1.first.first, s1.second.first) &&
           intersection.first >= std::min(s2.first.first, s2.second.first) &&
           intersection.first <= std::max(s2.first.first, s2.second.first) &&
           intersection.second >= std::min(s1.first.second, s1.second.second) &&
           intersection.second <= std::max(s1.first.second, s1.second.second) &&
           intersection.second >= std::min(s2.first.second, s2.second.second) &&
           intersection.second <= std::max(s2.first.second, s2.second.second);
}


/*!
 * Angle between vectors (p1, p2) and (p2, p3)
 * @param p1 First point of first vector
 * @param p2 Second point of first vector, first point of third vector
 * @param p3 Second point of the second vector
 * @return  Clockwise angle between two vectors in range (0, 2 * pi)
 */
double angle_between_vectors(point_t p1, point_t p2, point_t p3) {
    double x1 = p1.first - p2.first, y1 = p1.second - p2.second;
    double x2 = p3.first - p2.first, y2 = p3.second - p2.second;
    double dot = x1 * x2 + y1 * y2, det = x1 * y2 - y1 * x2;
    double angle = atan2(det, dot);
    if (angle < 0) {
        angle += 2 * M_PI;
    }
    return angle;
}

double distance_between_points(point_t p1, point_t p2) {
    return std::sqrt((p1.first - p2.first) * (p1.first - p2.first) +
                     (p1.second - p2.second) * (p1.second - p2.second));
}


// TODO: test this
point_t gps_coordinates_to_meters(point_t p) {
    // -465711, -5249470
    std::string zone_res;

    point_t res;
    res.second = p.second * METERS_IN_DEGREE;
    res.first = std::cos((p.second / 180.0) * M_PI) * p.first * METERS_IN_DEGREE;
    return res;
}

point_t meters_to_gps_coordinates(point_t p) {
    point_t res;

    res.second = p.second / METERS_IN_DEGREE;
    res.first = p.first / (std::cos(res.second / 180.0 * M_PI) * METERS_IN_DEGREE);
    return res;
}

int generate_random_number() {
    static std::random_device rd;
    static std::mt19937 generator(rd());
    static std::uniform_int_distribution<int> distribution(0, std::numeric_limits<int>::max());

    return distribution(generator);
}

bool polygon_convex(std::vector<point_t> polygon) {
    // Push a new node to use only one loop further
    polygon.push_back(polygon[1]);
    int neg_angles = 0, pos_angles = 0;
    for (size_t i = 0; i + 2 < polygon.size(); ++i) {
        double angle = M_PI - angle_between_vectors(polygon[i], polygon[i + 1], polygon[i + 2]);
        if (angle < 0 - 1e-5) {
            ++neg_angles;
        } else if (angle > 1e-5) {
            ++pos_angles;
        }
    }
    return neg_angles == 0 || pos_angles == 0;
}

double get_segment_rotation(segment_t segment) {
    // TODO: check this formula
    segment.second.first -= segment.first.first;
    segment.second.second -= segment.first.second;
    segment.first = {0, 0};


    if (segment.second.first == 0) {
        return segment.second.second > 0 ? M_PI_2 : -M_PI_2;
    }

    double angle = std::atan(segment.second.second / segment.second.first);
    if (segment.second.first < 0) {
        return angle + M_PI;
    }
    return angle;
}

/*!
 * Make the polygon be directed clockwise (i.e. when travelling from the first to last point,
 * the area inside of polygon is always on the right) for the convenience of working with it
 */
void make_polygon_clockwise(polygon_t &polygon) {
    if (polygon.size() <= 2) {
        return;
    }
    double angle = M_PI - angle_between_vectors(polygon[polygon.size() - 2], polygon[0], polygon[1]);
    for (size_t i = 0; i + 2 < polygon.size(); ++i) {
        angle += M_PI - angle_between_vectors(polygon[i], polygon[i + 1], polygon[i + 2]);
    }
    // If the outer angle of the whole polygon is -2 * pi -- we need to reverse it
    if (std::abs(-2 * M_PI - angle) < std::abs(2 * M_PI - angle)) {
        std::reverse(polygon.begin(), polygon.end());
    }
}

point_t gps_coordinates_to_meters(point_t p, point_t origin) {
    // TODO: This function ignores Earth curvature. This has to be replaced with a better calculation
    const double meters_in_long_degree = std::cos((origin.first / 180.0) * M_PI) * METERS_IN_DEGREE;
    return {p.second * meters_in_long_degree - origin.second * meters_in_long_degree, p.first * METERS_IN_DEGREE - origin.first * METERS_IN_DEGREE};
}

point_t meters_to_gps_coordinates(point_t p, point_t origin) {
    const double meters_in_long_degree = std::cos((origin.first / 180.0) * M_PI) * METERS_IN_DEGREE;
    return {(p.second + origin.first * METERS_IN_DEGREE) / METERS_IN_DEGREE, (p.first + origin.second * meters_in_long_degree) / meters_in_long_degree};
}


std::pair<double, double> solve_quadratic(double a, double b, double c) {
    if (a == 0) {
        return {-c / b, -c / b};
    }
    double d = b * b - 4 * a * c;
    if (d < 0) {
        return {FP_NAN, FP_NAN};
    }
    return {(-b + std::sqrt(d)) / (2 * a), (-b - std::sqrt(d)) / (2 * a)};
}


point_t segment_vertical_line_intersection(const segment_t &s, double x) {
    const double &x1 = s.first.first, &y1 = s.first.second, &x2 = s.second.first, &y2 = s.second.second;

    // TODO: maybe, use some other thing for the event when the segment is completely vertical
    // Perfectly vertical line
    if (s.first.first == s.second.first) {
        // Return just the middle point of the segment
        return {x, (y1 + y2) / 2};
    }

    return {x, -((y2 - y1) * (x2 - x) / (x2 - x1) - y2)};
}
