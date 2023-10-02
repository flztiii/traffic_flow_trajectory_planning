#include "shape.hpp"

ErrorType GetVerticesOfOrientedBoundingBox(const OrientedBoundingBox2D& obb, vec_E<Vecf<2>>* vertices) {
    //
    //    corner2   corner1
    // y<--   ________   x
    //        |      |   ^
    //        |      |   |
    //        |      |
    //        |______|
    //    corner3   corner4
    vertices->clear();
    vertices->reserve(4);
    double cos_theta = cos(obb.angle);
    double sin_theta = sin(obb.angle);
    Vecf<2> corner1(obb.x + 0.5 * obb.length * cos_theta + 0.5 * obb.width * sin_theta, obb.y + 0.5 * obb.length * sin_theta - 0.5 * obb.width * cos_theta);
    Vecf<2> corner2(obb.x + 0.5 * obb.length * cos_theta - 0.5 * obb.width * sin_theta, obb.y + 0.5 * obb.length * sin_theta + 0.5 * obb.width * cos_theta);
    Vecf<2> corner3(obb.x - 0.5 * obb.length * cos_theta - 0.5 * obb.width * sin_theta, obb.y - 0.5 * obb.length * sin_theta + 0.5 * obb.width * cos_theta);
    Vecf<2> corner4(obb.x - 0.5 * obb.length * cos_theta + 0.5 * obb.width * sin_theta, obb.y - 0.5 * obb.length * sin_theta - 0.5 * obb.width * cos_theta);
    vertices->push_back(corner1);
    vertices->push_back(corner2);
    vertices->push_back(corner3);
    vertices->push_back(corner4);
    return kSuccess;
}

bool CheckIfOrientedBoundingBoxIntersect(const OrientedBoundingBox2D& obb_a, const OrientedBoundingBox2D& obb_b) {
    vec_E<Vecf<2>> vertices_a, vertices_b;
    GetVerticesOfOrientedBoundingBox(obb_a, &vertices_a);
    GetVerticesOfOrientedBoundingBox(obb_b, &vertices_b);
    vec_E<Vecf<2>> axes;
    GetPerpendicularAxesOfOrientedBoundingBox(vertices_a, &axes);
    GetPerpendicularAxesOfOrientedBoundingBox(vertices_b, &axes);
    Vecf<2> proj_a, proj_b;
    double overlap_len;
    // double minoverlap = std::numeric_limits<double>::infinity();
    for (auto& axis : axes) {
        GetProjectionOnAxis(vertices_a, axis, &proj_a);
        GetProjectionOnAxis(vertices_b, axis, &proj_b);
        GetOverlapLength(proj_a, proj_b, &overlap_len);
        if (fabs(overlap_len) < kEPS) {  // shapes are not overlapping
        return false;
        }
    }
    return true;
}

ErrorType GetPerpendicularAxesOfOrientedBoundingBox(const vec_E<Vecf<2>>& vertices, vec_E<Vecf<2>>* axes) {
    // for obb 2d, two axes are enough
    Vecf<2> axis0, axis1;
    GetPerpendicularAxisOfOrientedBoundingBox(vertices, 0, &axis0);
    GetPerpendicularAxisOfOrientedBoundingBox(vertices, 1, &axis1);
    axes->push_back(axis0);
    axes->push_back(axis1);
    return kSuccess;
}

ErrorType GetProjectionOnAxis(const vec_E<Vecf<2>>& vertices, const Vecf<2>& axis, Vecf<2>* proj) {
    double min = std::numeric_limits<double>::infinity();
    double max = -std::numeric_limits<double>::infinity();
    double projection;
    for (auto& vertex : vertices) {
        projection = vertex.dot(axis);
        if (projection < min) {
        min = projection;
        }
        if (projection > max) {
        max = projection;
        }
    }
    *proj = Vecf<2>(min, max);
    return kSuccess;
}

ErrorType GetOverlapLength(const Vecf<2> a, const Vecf<2> b, double* len) {
    if (a.x() > b.y() || a.y() < b.x()) {
        *len = 0.0;
        return kSuccess;
    }

    *len = std::min(a.y(), b.y()) - std::max(a.x(), b.x());
    return kSuccess;
}

ErrorType GetPerpendicularAxisOfOrientedBoundingBox(const vec_E<Vecf<2>>& vertices, const int index, Vecf<2>* axis) {
    assert(index >= 0 && index < 4);
    Vecf<2> vec = vertices[index + 1] - vertices[index];
    double length = vec.norm();
    Vecf<2> normalized_vec = Vecf<2>::Zero();
    if (length > kEPS) normalized_vec = vec / length;
    // right hand normal vector
    (*axis)[0] = -normalized_vec[1];
    (*axis)[1] = normalized_vec[0];
    return kSuccess;
}

