#ifndef LANE_HPP_
#define LANE_HPP_

#include "spline.hpp"
#include "spline_generator.hpp"

#define LaneDegree 3
#define LaneDim 2

class Lane {
 public:
    typedef Spline<LaneDegree, LaneDim> SplineType;

    Lane() {}
    
    Lane(const SplineType& position_spline) {
        this->position_spline_ = position_spline;
        this->is_valid_ = true;
    }
    
    bool IsValid() const { 
        return this->is_valid_;
    }

    /**
     * @brief Set the parameterization of the lane
     * @param position spline, position spline with required degree
     */
    void set_position_spline(const SplineType& position_spline) {
        if (position_spline.vec_domain().empty()) {
            return;
        }
        this->position_spline_ = position_spline;
        this->is_valid_ = true;
    }

    /**
     * @brief Get curvature by arc length
     * @param arc_length, evaluation arc length
     * @param curvature, curvature returned
     * @param curvature, curvature derivative
     */
    ErrorType GetCurvatureByArcLength(const double& arc_length, double* curvature, double* curvature_derivative) const {
        if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2) {
            return kIllegalInput;
        }

        Vecf<LaneDim> vel, acc, jrk;
        GetDerivativeByArcLength(arc_length, 1, &vel);
        // printf("vel: (%lf, %lf) at arc length %lf.\n", vel[0], vel[1], arc_length);
        GetDerivativeByArcLength(arc_length, 2, &acc);
        // printf("acc: (%lf, %lf) at arc length %lf.\n", acc[0], acc[1], arc_length);
        GetDerivativeByArcLength(arc_length, 3, &jrk);
        // printf("jrk: (%lf, %lf) at arc length %lf.\n", jrk[0], jrk[1], arc_length);

        double c0 = vel[0] * acc[1] - vel[1] * acc[0];
        double c1 = vel.norm();
        *curvature = c0 / (c1 * c1 * c1);
        *curvature_derivative = ((acc[0] * acc[1] + vel[0] * jrk[1] - acc[1] * acc[0] - vel[1] * jrk[0]) / c1 * c1 * c1 - 3 * c0 * (vel[0] * acc[0] + vel[1] * acc[1]) / (c1 * c1 * c1 * c1 * c1));
        return kSuccess;
    }

    ErrorType GetCurvatureByArcLength(const double& arc_length, double* curvature) const {
        if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2) {
            return kIllegalInput;
        }

        Vecf<LaneDim> vel, acc;
        GetDerivativeByArcLength(arc_length, 1, &vel);
        GetDerivativeByArcLength(arc_length, 2, &acc);

        double c0 = vel[0] * acc[1] - vel[1] * acc[0];
        double c1 = vel.norm();
        *curvature = c0 / (c1 * c1 * c1);
        return kSuccess;
    }

    /**
     * @brief Get derivative by arc length (d = 0 means position evaluation)
     * @param d, derivative to take
     * @param derivative, derivative returned
     */
    ErrorType GetDerivativeByArcLength(const double arc_length, const int d, Vecf<LaneDim>* derivative) const {
        return this->position_spline_.evaluate(arc_length, d, derivative);
    }

    ErrorType GetPositionByArcLength(const double arc_length, Vecf<LaneDim>* derivative) const {
        return this->position_spline_.evaluate(arc_length, derivative);
    }

    ErrorType GetTangentVectorByArcLength(const double arc_length, Vecf<LaneDim>* tangent_vector) const {
        if (CheckInputArcLength(arc_length) != kSuccess) {
            return kIllegalInput;
        }

        Vecf<LaneDim> vel;
        GetDerivativeByArcLength(arc_length, 1, &vel);

        if (vel.norm() < kEPS) {
            return kWrongStatus;
        }

        *tangent_vector = vel / vel.norm();
        return kSuccess;
    }

    ErrorType GetNormalVectorByArcLength(const double arc_length, Vecf<LaneDim>* normal_vector) const {
        if (CheckInputArcLength(arc_length) != kSuccess) {
            return kIllegalInput;
        }

        Vecf<LaneDim> vel;
        GetDerivativeByArcLength(arc_length, 1, &vel);

        if (vel.norm() < kEPS) {
            return kWrongStatus;
        }

        Vecf<LaneDim> tangent_vector = vel / vel.norm();
        *normal_vector = rotate_vector_2d(tangent_vector, M_PI / 2.0);
        return kSuccess;
    }

    ErrorType GetOrientationByArcLength(const double arc_length, double* angle) const {
        if (CheckInputArcLength(arc_length) != kSuccess) {
            return kIllegalInput;
        }

        Vecf<LaneDim> vel;
        GetDerivativeByArcLength(arc_length, 1, &vel);

        if (vel.norm() < kEPS) {
            return kWrongStatus;
        }

        Vecf<LaneDim> tangent_vector = vel / vel.norm();
        *angle = vec2d_to_angle(tangent_vector);
        return kSuccess;
    }

    ErrorType GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position, double* arc_length) const {
        if (!IsValid()) {
            return kWrongStatus;
        }

        static constexpr int kMaxCnt = 12;
        static constexpr double kMaxDistSquare = 900.0;

        const double val_lb = this->position_spline_.begin();
        const double val_ub = this->position_spline_.end();
        double step = (val_ub - val_lb) * 0.5;

        double s1 = val_lb;
        double s2 = val_lb + step;
        double s3 = val_ub;
        double initial_guess = s2;

        // printf("[XXX]s1 = %lf, s2 = %lf, s3 = %lf\n", s1, s2, s3);

        Vecf<LaneDim> start_pos, mid_pos, final_pos;
        this->position_spline_.evaluate(s1, &start_pos);
        this->position_spline_.evaluate(s2, &mid_pos);
        this->position_spline_.evaluate(s3, &final_pos);

        // ~ Step I: use binary search to find a initial guess
        double d1 = (start_pos - vec_position).squaredNorm();
        double d2 = (mid_pos - vec_position).squaredNorm();
        double d3 = (final_pos - vec_position).squaredNorm();

        for (int i = 0; i < kMaxCnt; ++i) {
            // printf("[XXX] - it = %d - s1 = %lf, s2 = %lf, s3 = %lf\n", i, s1, s2, s3);
            double min_dis = std::min(std::min(d1, d2), d3);
            if (min_dis < kMaxDistSquare) {
                if (min_dis == d1) {
                    initial_guess = s1;
                } else if (min_dis == d2) {
                    initial_guess = s2;
                } else if (min_dis == d3) {
                    initial_guess = s3;
                } else {
                    throw;
                }
                break;
            }
            step *= 0.5;
            if (min_dis == d1) {
                initial_guess = s1;
                s3 = s2;
                s2 = s1 + step;
                this->position_spline_.evaluate(s2, &mid_pos);
                this->position_spline_.evaluate(s3, &final_pos);
                d2 = (mid_pos - vec_position).squaredNorm();
                d3 = (final_pos - vec_position).squaredNorm();
            } else if (min_dis == d2) {
                initial_guess = s2;
                s1 = s2 - step;
                s3 = s2 + step;
                this->position_spline_.evaluate(s1, &start_pos);
                this->position_spline_.evaluate(s3, &final_pos);
                d1 = (start_pos - vec_position).squaredNorm();
                d3 = (final_pos - vec_position).squaredNorm();
            } else if (min_dis == d3) {
                initial_guess = s3;
                s1 = s2;
                s2 = s3 - step;
                this->position_spline_.evaluate(s1, &start_pos);
                this->position_spline_.evaluate(s2, &mid_pos);
                d1 = (start_pos - vec_position).squaredNorm();
                d2 = (mid_pos - vec_position).squaredNorm();
            } else {
                printf("[Lane]GetArcLengthByVecPosition - d1: %lf, d2: %lf, d3: %lf, "
                "min_dis: %lf\n", d1, d2, d3, min_dis);
                throw;
            }
        }
        // printf("[XXX]initial_guess = %lf\n", initial_guess);

        // ~ Step II: use Newton's method to find the local minimum
        GetArcLengthByVecPositionWithInitialGuess(vec_position, initial_guess, arc_length);
        return kSuccess;
    }

    ErrorType GetArcLengthByVecPositionWithInitialGuess( const Vecf<LaneDim>& vec_position, const double& initial_guess, double* arc_length) const {
        if (!IsValid()) {
            return kWrongStatus;
        }

        const double val_lb = this->position_spline_.begin();
        const double val_ub = this->position_spline_.end();

        // ~ use Newton's method to find the local minimum
        static constexpr double epsilon = 1e-3;
        static constexpr int kMaxIter = 8;
        double x = std::min(std::max(initial_guess, val_lb), val_ub);
        Vecf<LaneDim> p, dp, ddp, tmp_vec;

        for (int i = 0; i < kMaxIter; ++i) {
            this->position_spline_.evaluate(x, 0, &p);
            this->position_spline_.evaluate(x, 1, &dp);
            this->position_spline_.evaluate(x, 2, &ddp);

            tmp_vec = p - vec_position;
            double f_1 = tmp_vec.dot(dp);
            double f_2 = dp.dot(dp) + tmp_vec.dot(ddp);
            double dx = -f_1 / f_2;

            if (std::fabs(dx) < epsilon) {
                break;
            }

            if (x + dx > val_ub) {
                x = val_ub;
                // printf(
                //     "[Lane]GetArcLengthByVecPosition - Out of range, use upper
                //     bound\n");
                break;
            } else if (x + dx < val_lb) {
                x = val_lb;
                // printf(
                //     "[Lane]GetArcLengthByVecPosition - Out of range, use lower
                //     bound\n");
                break;
            }

            x += dx;
        }

        *arc_length = x;

        return kSuccess;
    }

    ErrorType CheckInputArcLength(const double arc_length) const {
        if (!IsValid()) {
            printf("[CheckInputArcLength]Quering invalid lane.\n");
            return kWrongStatus;
        }
        if (arc_length < this->position_spline_.begin() - kEPS ||
            arc_length > this->position_spline_.end() + kEPS) {
            return kIllegalInput;
        }
        return kSuccess;
    }

    SplineType position_spline() const {
        return this->position_spline_;
    }

    double begin() const {
        return this->position_spline_.begin();
    }

    double end() const {
        return this->position_spline_.end();
    }

    void print() const {
        this->position_spline_.print();
    }

 private:
    SplineType position_spline_;
    bool is_valid_ = false;
};

// 根据采样点插值生成道路
ErrorType GetLaneBySampleInterpolation(const vec_Vecf<LaneDim>& samples, const std::vector<double>& para, Lane* lane);

// 根据采样点插值生成道路
ErrorType GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples, Lane* lane);

// 在道路上进行采样
ErrorType SampleLane(const Lane &lane, const double &s0, const double &s1, const double &step, vec_E<Vecf<2>> *samples, double *accum_dist);

// 计算道路与给定状态的距离
ErrorType GetDistanceToLanesUsing3DofState(const Vec3f &state, const Lane &lane,std::tuple<double, double, double> &res);

// 可视化道路
void visualizeLane(const Lane &lane, const ros::Publisher &publisher);

#endif  // LANE_HPP_
