#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "calculation.hpp"
#include "shape.hpp"

class VehicleParam {
 public:
    inline double width() const { 
        return this->width_; 
    }

    inline double length() const { 
        return this->length_; 
    }

    inline double wheel_base() const { 
        return this->wheel_base_; 
    }

    inline double front_suspension() const { 
        return this->front_suspension_; 
    }

    inline double rear_suspension() const { 
        return this->rear_suspension_; 
    }

    inline double max_steering_angle() const { 
        return this->max_steering_angle_; 
    }

    inline double max_curvature() const {
        return this->max_curvature_;
    }

    inline double max_longitudinal_acc() const { 
        return this->max_longitudinal_acc_; 
    }

    inline double max_longitudinal_jerk() const { 
        return this->max_longitudinal_jerk_;
    }

    inline double max_lateral_vel() const { 
        return this->max_lateral_vel_;
    }

    inline double max_lateral_acc() const { 
        return this->max_lateral_acc_;
    }

    inline double max_lateral_jerk() const { 
        return this->max_lateral_jerk_;
    }

    inline double d_cr() const { 
        return this->d_cr_; 
    }

    inline void set_width(const double val) { 
        this->width_ = val; 
    }

    inline void set_length(const double val) { 
        this->length_ = val; 
    }

    inline void set_wheel_base(const double val) { 
        this->wheel_base_ = val; 
    }

    inline void set_front_suspension(const double val) {
        this->front_suspension_ = val;
    }

    inline void set_rear_suspension(const double val) { 
        this->rear_suspension_ = val; 
    }

    inline void set_max_steering_angle(const double val) {
        this->max_steering_angle_ = val;
    }
    inline void set_max_longitudinal_acc(const double val) {
        this->max_longitudinal_acc_ = val;
    }

    inline void set_max_lateral_acc(const double val) { 
        this->max_lateral_acc_ = val; 
    }

    inline void set_max_lateral_vel(const double val) { 
        this->max_lateral_vel_ = val; 
    }

    inline void set_d_cr(const double val) { 
        this->d_cr_ = val; 
    }

    /**
     * @brief Print info
     */
    void print() const {
        printf("VehicleParam:\n");
        printf(" -- width:\t %lf.\n", this->width_);
        printf(" -- length:\t %lf.\n", this->length_);
        printf(" -- wheel_base:\t %lf.\n", this->wheel_base_);
        printf(" -- front_suspension:\t %lf.\n", this->front_suspension_);
        printf(" -- rear_suspension:\t %lf.\n", this->rear_suspension_);
        printf(" -- d_cr:\t %lf.\n", this->d_cr_);
        printf(" -- max_steering_angle:\t %lf.\n", this->max_steering_angle_);
        printf(" -- max_longitudinal_acc:\t %lf.\n", this->max_longitudinal_acc_);
        printf(" -- max_lateral_acc:\t %lf.\n", this->max_lateral_acc_);
    };

 private:
    double width_ = 1.90;
    double length_ = 4.88;
    double wheel_base_ = 2.85;
    double front_suspension_ = 0.93;
    double rear_suspension_ = 1.10;
    double max_steering_angle_ = 45.0;
    double max_curvature_ = 0.16;
    double max_lateral_vel_ = 2.0;
    double max_longitudinal_acc_ = 2.0;
    double max_lateral_acc_ = 2.0;
    double max_longitudinal_jerk_ = 2.0;
    double max_lateral_jerk_ = 2.0;

    double d_cr_ = 1.34;  // length between geometry center and rear axle
};

class Vehicle {
 public:
    Vehicle() {};

    Vehicle(const VehicleParam &param, const State &state) {
        this->param_ = param;
        this->state_ = state;
    };

    Vehicle(const int &id, const VehicleParam &param, const State &state) {
        this->id_ = id;
        this->param_ = param;
        this->state_ = state;
    };

    Vehicle(const int &id, const std::string &subclass, const VehicleParam &param, const State &state) {
        this->id_ = id;
        this->subclass_ = subclass;
        this->param_ = param;
        this->state_ = state;
    };

    inline int id() const { 
        return this->id_; 
    }

    inline std::string subclass() const { 
        return this->subclass_; 
    }

    inline VehicleParam param() const { 
        return this->param_; 
    }

    inline State state() const { 
        return this->state_; 
    }

    inline std::string type() const { 
        return this->type_; 
    }

    inline void set_id(const int &id) { 
        this->id_ = id; 
    }

    inline void set_subclass(const std::string &subclass) {
        this->subclass_ = subclass;
    }

    inline void set_type(const std::string &type) { 
        this->type_ = type; 
    }

    inline void set_param(const VehicleParam &in) { 
        this->param_ = in; 
    }

    inline void set_state(const State &in) { 
        this->state_ = in; 
    }

    /**
     * @brief Get 3-DoF vehicle state at center of rear axle, x-y-yaw
     *
     * @return Vec3f 3-dof vehicle state
     */
    Vec3f Ret3DofState() const {
        return Vec3f(state_.vec_position(0), state_.vec_position(1), state_.angle);
    };

    /**
     * @brief Get 2D OBB of the vehicle
     *
     * @return OrientedBoundingBox2D OBB
     */
    OrientedBoundingBox2D RetOrientedBoundingBox() const {
        OrientedBoundingBox2D obb;
        double cos_theta = cos(state_.angle);
        double sin_theta = sin(state_.angle);
        obb.x = state_.vec_position(0) + param_.d_cr() * cos_theta;
        obb.y = state_.vec_position(1) + param_.d_cr() * sin_theta;
        obb.angle = state_.angle;
        obb.width = param_.width();
        obb.length = param_.length();
        return obb;
    }

    /**
     * @brief Return color in jet colormap using mapping function
     *
     * @param vertices pointer of vertice container
     * @return ErrorType
     */
    ErrorType RetVehicleVertices(vec_E<Vec2f> *vertices) const {
        double angle = this->state_.angle;

        double cos_theta = cos(angle);
        double sin_theta = sin(angle);

        double c_x = this->state_.vec_position(0) + this->param_.d_cr() * cos_theta;
        double c_y = this->state_.vec_position(1) + this->param_.d_cr() * sin_theta;

        double d_wx = this->param_.width() / 2 * sin_theta;
        double d_wy = this->param_.width() / 2 * cos_theta;
        double d_lx = this->param_.length() / 2 * cos_theta;
        double d_ly = this->param_.length() / 2 * sin_theta;

        // Counterclockwise from left-front vertex
        vertices->push_back(Vec2f(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
        // vertices->push_back(Vec2f(c_x - d_wx, c_y + d_wy));
        vertices->push_back(Vec2f(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
        // vertices->push_back(Vec2f(c_x - d_lx, c_y - d_ly));
        vertices->push_back(Vec2f(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
        // vertices->push_back(Vec2f(c_x + d_wx, c_y - d_wy));
        vertices->push_back(Vec2f(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
        // vertices->push_back(Vec2f(c_x + d_lx, c_y + d_ly));

        return kSuccess;
    }

    /**
     * @brief Return front and rear points on longitudinal axle
     *
     * @param vertices
     * @return ErrorType
     */
    ErrorType RetBumperVertices(std::array<Vec2f, 2> *vertices) const {
        double cos_theta = cos(this->state_.angle);
        double sin_theta = sin(this->state_.angle);

        double c_x = this->state_.vec_position(0) + this->param_.d_cr() * cos_theta;
        double c_y = this->state_.vec_position(1) + this->param_.d_cr() * sin_theta;

        double d_lx = this->param_.length() / 2.0 * cos_theta;
        double d_ly = this->param_.length() / 2.0 * sin_theta;

        (*vertices)[0] = Vec2f(c_x - d_lx, c_y - d_ly);
        (*vertices)[1] = Vec2f(c_x + d_lx, c_y + d_ly);

        return kSuccess;
    };

    /**
     * @brief Return 3-DoF state at geometry center, x-y-yaw
     *
     * @param state pointer of state
     * @return ErrorType
     */
    ErrorType Ret3DofStateAtGeometryCenter(Vec3f *state) const {
        double cos_theta = cos(this->state_.angle);
        double sin_theta = sin(this->state_.angle);
        double x = this->state_.vec_position(0) + this->param_.d_cr() * cos_theta;
        double y = this->state_.vec_position(1) + this->param_.d_cr() * sin_theta;
        (*state)(0) = x;
        (*state)(1) = y;
        (*state)(2) = this->state_.angle;
        return kSuccess;
    };

    /**
     * @brief Print info
     */
    void print() const {
        printf("\nVehicle:\n");
        printf(" -- ID:\t%d\n", id_);
        printf(" -- Subclass:\t%s\n", subclass_.c_str());
        param_.print();
        state_.print();
    };

 private:
    int id_{kInvalidAgentId};
    std::string subclass_;
    std::string type_;
    VehicleParam param_;
    State state_;
};

/**
 * @brief Vehicle control signal, 2 modes embedded
 * @brief open-loop: use desired state
 * @brief closed-loop: use longitudinal acc and steering rate
 */
struct VehicleControlSignal {
    double acc = 0.0;
    double steer_rate = 0.0;
    bool is_openloop = false;
    State state;

    /**
     * @brief Default constructor
     */
    VehicleControlSignal() {};

    /**
     * @brief Construct a new Vehicle Control Signal object
     *
     * @param acc longitudinal acc, m/s^2
     * @param steer_rate steering rate, rad/s
     */
    VehicleControlSignal(double acc, double steer_rate): acc(acc), steer_rate(steer_rate), is_openloop(false) {};

    /**
     * @brief Construct a new Vehicle Control Signal object
     *
     * @param state desired state
     */
    VehicleControlSignal(State state): acc(0.0), steer_rate(0.0), is_openloop(true), state(state) {};
};

#endif  // VEHICLE_HPP_
