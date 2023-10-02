#ifndef COMMON_HPP_
#define COMMON_HPP_

#include "basic.hpp"

// 状态
struct State {
    double time_stamp{0.0};
    Vecf<2> vec_position{Vecf<2>::Zero()};
    double angle{0.0};
    double curvature{0.0};
    double velocity{0.0};
    double acceleration{0.0};
    double steer{0.0};
    
    void print() const {
        printf("State:\n");
        printf(" -- time_stamp: %lf.\n", time_stamp);
        printf(" -- vec_position: (%lf, %lf).\n", vec_position[0], vec_position[1]);
        printf(" -- angle: %lf.\n", angle);
        printf(" -- curvature: %lf.\n", curvature);
        printf(" -- velocity: %lf.\n", velocity);
        printf(" -- acceleration: %lf.\n", acceleration);
        printf(" -- steer: %lf.\n", steer);
    }

    Vec3f ToXYTheta() const {
        return Vec3f(vec_position(0), vec_position(1), angle);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 横向行为
enum class LateralBehavior {
    kUndefined = 0,
    kLaneKeeping,
    kLaneChangeLeft,
    kLaneChangeRight,
};

#endif  // COMMON_HPP_