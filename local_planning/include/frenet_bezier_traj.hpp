#ifndef FRENET_BEZIER_TRAJ_H__
#define FRENET_BEZIER_TRAJ_H__

#include "spline.hpp"
#include "state_transformer.hpp"
#include "trajectory.hpp"

template<int TrajectoryDegree, int TrajectoryDim>
class FrenetBezierTrajectory : public FrenetTrajectory {
 public:
    using BezierTrajectory = BezierSpline<TrajectoryDegree, TrajectoryDim>;

    FrenetBezierTrajectory() {}

    FrenetBezierTrajectory(const BezierTrajectory& bezier_spline, const StateTransformer& stf): bezier_spline_(bezier_spline), stf_(stf), is_valid_(true) {}

    double begin() const override { 
        return bezier_spline_.begin(); 
    }
    
    double end() const override { 
        return bezier_spline_.end(); 
    }
    
    bool IsValid() const override { 
        return is_valid_; 
    }
    
    ErrorType GetState(const double& t, State* state) const override {
        if (t < begin() - kEPS || t > end() + kEPS) {
            return kWrongStatus;
        }
        FrenetState fs;
        if (GetFrenetState(t, &fs) != kSuccess) {
            return kWrongStatus;
        }
        if (stf_.GetStateFromFrenetState(fs, state) != kSuccess) {
            return kWrongStatus;
        }
        state->velocity = std::max(0.0, state->velocity);
        return kSuccess;
    }

    ErrorType GetFrenetState(const double& t, FrenetState* fs) const override {
        if (t < begin() - kEPS || t > end() + kEPS) {
            return kWrongStatus;
        }
        Vecf<2> pos, vel, acc;
        bezier_spline_.evaluate(t, 0, &pos);
        bezier_spline_.evaluate(t, 1, &vel);
        bezier_spline_.evaluate(t, 2, &acc);
        // ~ frenet bezier by default works lateral independent mode
        fs->time_stamp = t;
        fs->Load(Vec3f(pos[0], vel[0], acc[0]), Vec3f(pos[1], vel[1], acc[1]), FrenetState::kInitWithDt);
        if (!fs->is_ds_usable) {
            fs->Load(Vec3f(pos[0], 0.0, 0.0), Vec3f(pos[1], 0.0, 0.0), FrenetState::kInitWithDs);
        }
        return kSuccess;
    }

    std::vector<double> variables() const override {
        // TODO
        return std::vector<double>();
    }

    BezierTrajectory bezier_spline() const {
        return this->bezier_spline_;
    }

    StateTransformer state_transformer() const {
        return this->stf_;
    }

    void set_variables(const std::vector<double>& variables) override {
        // TODO
    }

    virtual void Jerk(double* j_lon, double* j_lat) const override {
        // TODO
    }

 private:
    BezierTrajectory bezier_spline_;
    StateTransformer stf_;
    bool is_valid_ = false;
};

#endif  // FRENET_BEZIER_TRAJ_H__