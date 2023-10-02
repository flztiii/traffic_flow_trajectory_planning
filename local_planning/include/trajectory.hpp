#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include "frenet_state.hpp"

class Trajectory {
 public:
    virtual ~Trajectory() = default;
    virtual ErrorType GetState(const double& t, State* state) const = 0;
    virtual double begin() const = 0;
    virtual double end() const = 0;
    virtual bool IsValid() const = 0;
    // optimization-related interface
    virtual std::vector<double> variables() const = 0;  // return a copy of variables
    virtual void set_variables(const std::vector<double>& variables) = 0;
};

class FrenetTrajectory : public Trajectory {
 public:
    virtual ~FrenetTrajectory() = default;
    virtual ErrorType GetState(const double& t, State* state) const = 0;
    virtual ErrorType GetFrenetState(const double& t, FrenetState* fs) const = 0;
    virtual double begin() const = 0;
    virtual double end() const = 0;
    virtual bool IsValid() const = 0;
    virtual std::vector<double> variables() const = 0;  // return a copy of variables
    virtual void set_variables(const std::vector<double>& variables) = 0;
    // * Frenet interfaces
    virtual void Jerk(double* j_lon, double* j_lat) const = 0;
};

#endif  // TRAJECTORY_HPP_