#ifndef PATH_HPP
#define PATH_HPP

#include <pathing/spline.hpp>

/**
 * @brief A structure representing a 2D coordinate
*/
struct Point {
    double x, y;
    Point() = default;
    Point(double x, double y): x(x), y(y) {}
};

/**
 * @brief The Path class to interpolate between waypoints.
 * @attention The Path class uses cubic splines to interpolate between waypoints. Parametrization is done
   internally and performed automatically within the constructor.
*/
class Path {
public:
    /**
     * @brief The x and y coordinates of the path.
    */
    std::vector<Point> points;

    /**
     * @brief The default constructor for the Path class.
    */
    Path() = default;

    /**
     * @brief The constructor for the Path class.
     * @param x The x coordinates of the waypoints.
     * @param y The y coordinates of the waypoints.
     * @param num_points The number of points to interpolate in total.
     * @attention x and y are the ACTUAL coordinates; do NOT parametrize as this is done automatically
       within the constructor.
    */
    Path(std::vector<double> x, std::vector<double> y, int num_points = 100);

    /**
     * @brief Get the segment of the path that the point is on.
     * @param point The point to check.
     * @return The segment of the path that the point is on.
     */
    int get_segment(Point point);

    /**
     * @brief Get the segment of the path that the point is on.
     * @param x The x-coordinate of the point.
     * @param y The y-coordinate of the point.
     * @return The segment of the path that the point is on.
     */
    int get_segment(double x, double y);

    /**
     * @brief Get the distance to the end of the path from the point.
     * @param point The point to check.
     * @return The distance to the end of the path from the point.
     */
    double distance_to_end(Point point);

    /**
     * @brief Get the distance to the end of the path from the point.
     * @param x The x-coordinate of the point.
     * @param y The y-coordinate of the point.
     * @return The distance to the end of the path from the point.
     */
    double distance_to_end(double x, double y);

    Point closest_point(double x, double y);
};

#endif