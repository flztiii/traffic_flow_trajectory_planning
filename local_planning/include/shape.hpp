#ifndef SHAPE_HPP_
#define SHAPE_HPP_

#include "calculation.hpp"

struct OrientedBoundingBox2D {
    double x;
    double y;
    double angle;
    double width;
    double length;

    /**
     * @brief Default constructor
     */
    OrientedBoundingBox2D() {};

    /**
     * @brief Construct a new Oriented Bounding Box 2 D object
     *
     * @param x_ x of center of obb
     * @param y_ y of center of obb
     * @param angle_ angle between x-positive and longitudinal axis of obb
     * @param width_ width of obb
     * @param length_ length of obb
     */
    OrientedBoundingBox2D(const double x_, const double y_, const double angle_, const double width_, const double length_): x(x_), y(y_), angle(angle_), width(width_), length(length_) {};
};

/**
   * @brief Get the Vertices Of OrientedBoundingBox object
   *
   * @param obb Input OBB
   * @param vertices Output vertice vector
   * @return ErrorType
   */
ErrorType GetVerticesOfOrientedBoundingBox(const OrientedBoundingBox2D& obb, vec_E<Vecf<2>>* vertices);

bool CheckIfOrientedBoundingBoxIntersect(const OrientedBoundingBox2D& obb_a, const OrientedBoundingBox2D& obb_b);

ErrorType GetPerpendicularAxesOfOrientedBoundingBox(const vec_E<Vecf<2>>& vertices, vec_E<Vecf<2>>* axes);

ErrorType GetProjectionOnAxis(const vec_E<Vecf<2>>& vertices, const Vecf<2>& axis, Vecf<2>* proj);

ErrorType GetOverlapLength(const Vecf<2> a, const Vecf<2> b, double* len);

ErrorType GetPerpendicularAxisOfOrientedBoundingBox(const vec_E<Vecf<2>>& vertices, const int index, Vecf<2>* axis);

#endif  // SHAPE_HPP_