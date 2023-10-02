#ifndef TRAJECTORY_PLANNING_HPP_
#define TRAJECTORY_PLANNING_HPP_

#include "semantic.hpp"
#include "spline.hpp"
#include "ooqp_interface.hpp"
#include <boost/icl/interval_set.hpp>
#include <Eigen/SparseCore>
#include <queue>
#include "frenet_bezier_traj.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// 行为规划
class TrajectoryPlanner {
 public:
    // 构造函数
    TrajectoryPlanner(double max_time_horizon) {
        this->time_stamps_ = {0.0, 0.4, 0.9, 1.5, 2.2, 3.0, 4.0, 5.0};
        if (max_time_horizon + kEPS < this->time_stamps_.back()) {
            throw;
        }
        this->lane_width_ = 3.5;
        this->max_longitude_vel_ = 20.0;
        this->response_time_ = 1.0;
        this->min_valid_num_ = 1;
        ros::NodeHandle nh("~");
        this->debug_trajectories_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/local_planning/debug_trajectories_vis", 10, true);
    };

    // 析构函数
    ~TrajectoryPlanner() {};

    template<int N_DEG, int N_DIM>
    ErrorType planOnce(const SemanticVehicle &semantic_ego_vehicle, const SemanticLaneSet &semantic_lane_set, const SemanticVehicleSet &surr_semantic_vehicle_set, const LateralBehavior &desire_behavior, FrenetBezierTrajectory<N_DEG,N_DIM> &planned_trajectory) const {
        // 进行栅格生成
        std::vector<std::vector<Voxel>> voxels;
        if (this->voxelGeneration(semantic_ego_vehicle, semantic_lane_set, surr_semantic_vehicle_set, voxels) != kSuccess) {
            std::cout << "generate voxels failed" << std::endl;
            return kWrongStatus;
        }
        // 进行图构建
        std::vector<std::vector<std::shared_ptr<Node>>> voxel_graph;
        if (this->buildGraph(semantic_ego_vehicle, voxels, voxel_graph) != kSuccess) {
            std::cout << "build voxel graph failed" << std::endl;
            return kWrongStatus;
        }
        // 进行栅格序列搜索
        std::vector<std::pair<std::vector<Node>, double>> voxel_sequences_with_cost;
        if (this->graphSearch(voxel_graph, voxel_sequences_with_cost) != kSuccess){
            std::cout << "voxel graph search failed" << std::endl;
            return kWrongStatus;
        }
        // 进行栅格修正
        if (this->voxelRevise(voxels, semantic_ego_vehicle, voxel_sequences_with_cost)!= kSuccess) {
            std::cout << "voxel sequences revise failed" << std::endl;
            return kWrongStatus;
        }
        // 根据栅格序列使用优化方法进行轨迹生成
        std::vector<std::pair<LateralBehavior, FrenetBezierTrajectory<N_DEG,N_DIM>>> frenet_bezier_splines;
        if (this->trajectoryGeneration<N_DEG,N_DIM>(semantic_ego_vehicle, surr_semantic_vehicle_set, semantic_lane_set, voxel_sequences_with_cost, frenet_bezier_splines) != kSuccess){
            std::cout << "no available trajectory generated" << std::endl;
            return kWrongStatus;
        }
        // 进行轨迹选择
        std::cout << "desire_behavior: " << static_cast<int>(desire_behavior) << std::endl;
        if (frenet_bezier_splines.size() == 1) {
            // 只有一条轨迹，直接选择
            planned_trajectory = frenet_bezier_splines.front().second;
        } else {
            // 进行选择
            // 首先判断desire_behavior是否定义
            if (desire_behavior == LateralBehavior::kUndefined) {
                // 未定义，全状态可行
                // 进行最优评价
                bool find_trajectory = false;
                double min_cost = std::numeric_limits<double>::max();
                for (int i = 0; i < frenet_bezier_splines.size(); i++) {
                    for (int j = 0; j < voxel_sequences_with_cost.size(); j++) {
                        if (voxel_sequences_with_cost.at(j).first.back().voxel_.behavior_ == frenet_bezier_splines.at(i).first) {
                            double cur_cost = voxel_sequences_with_cost.at(j).second;
                            std::cout << "behaivor " << static_cast<int>(frenet_bezier_splines.at(i).first) << " cost is " << cur_cost << std::endl;
                            if (cur_cost < min_cost) {
                                min_cost = cur_cost;
                                planned_trajectory = frenet_bezier_splines.at(i).second;
                                find_trajectory = true;
                                break;
                            }
                        }
                    }
                }
                if (!find_trajectory) {
                    throw;
                }
            } else {
                // 已经定义，优先选择desire行为，否则选择道路保持
                std::vector<double> costs;
                costs.resize(frenet_bezier_splines.size());
                for (int i = 0; i < frenet_bezier_splines.size(); i++) {
                    if (frenet_bezier_splines.at(i).first == desire_behavior) {
                        costs.at(i) = -1.0;
                    } else if (frenet_bezier_splines.at(i).first == LateralBehavior::kLaneKeeping) {
                        costs.at(i) = 0.0;
                    } else {
                        costs.at(i) = 1.0;
                    }
                }
                auto min_cost_iter = std::min_element(costs.begin(), costs.end());
                int min_cost_index = min_cost_iter - costs.begin();
                planned_trajectory = frenet_bezier_splines.at(min_cost_index).second;
            }
        }
        return kSuccess;
    }
 
 private:
    std::vector<double> time_stamps_;
    double lane_width_;
    double max_longitude_vel_;
    double response_time_;
    int min_valid_num_;
    ros::Publisher debug_trajectories_vis_;

    // 定义栅格
    struct Voxel {
        double s_center_;
        double d_center_;
        double t_center_;
        double s_len_;
        double d_len_;
        double t_len_;
        LateralBehavior behavior_;
    };

    // 定义节点
    struct Node {
        Voxel voxel_;
        std::vector<std::pair<std::shared_ptr<Node>, double>> parents_;
        std::vector<std::pair<std::shared_ptr<Node>, double>> children_;
        bool is_open_{false};
        bool is_closed_{false};
        double cost_{0.0};
        std::shared_ptr<Node> pre_node_ptr_{nullptr};

        bool operator<(const std::shared_ptr<Node> node) const {
            return this->cost_ < node->cost_;
        };

        bool operator>(const std::shared_ptr<Node> node) const {
            return this->cost_ > node->cost_;
        }

        void open() {
            this->is_open_ = true;
            this->is_closed_ = false;
        }

        void close() {
            this->is_open_ = false;
            this->is_closed_ = true;        
        }
    };

    // 进行期望状态生成
    template <int N_DIM>
    ErrorType generateExpectedState(const SemanticVehicle &semantic_ego_vehicle, const SemanticVehicleSet &surr_semantic_vehicle_set, const SemanticLaneSet &semantic_lane_set, const std::vector<Node> &voxel_sequence, vec_E<Vecf<N_DIM>> &expected_end_pos, vec_E<Vecf<N_DIM>> &expected_end_vel) const {
        if (N_DIM != 2) {
            throw;
        }
        // 初始化
        expected_end_pos.resize(voxel_sequence.size());
        expected_end_vel.resize(voxel_sequence.size());
        // 首先得到自身车道的id
        int ego_lane_id = semantic_ego_vehicle.nearest_lane_id;
        // 得到左右车道的id
        int left_lane_id = semantic_lane_set.semantic_lanes.at(ego_lane_id).l_lane_id;
        int right_lane_id = semantic_lane_set.semantic_lanes.at(ego_lane_id).r_lane_id;
        // 得到frenet变换
        StateTransformer stf(semantic_ego_vehicle.lane);
        FrenetState init_ego_vehicle_fs;
        if (stf.GetFrenetStateFromState(semantic_ego_vehicle.vehicle.state(), &init_ego_vehicle_fs) != kSuccess) {
            throw;
        }
        // 得到车辆的初始s，初始s速度
        double init_s = init_ego_vehicle_fs.vec_s[0];
        double init_ds = init_ego_vehicle_fs.vec_s[1];

        for (int frame = 0; frame < voxel_sequence.size(); frame++) {
            Node current_voxel_node = voxel_sequence.at(frame);
            // 确定横向最终期望状态(横向位置)
            if (voxel_sequence.back().voxel_.behavior_ == LateralBehavior::kLaneKeeping) {
                expected_end_pos.at(frame)[1] = 0.0;
            } else if (voxel_sequence.back().voxel_.behavior_ == LateralBehavior::kLaneChangeLeft) {
                expected_end_pos.at(frame)[1] = this->lane_width_;
            } else if (voxel_sequence.back().voxel_.behavior_ == LateralBehavior::kLaneChangeRight) {
                expected_end_pos.at(frame)[1] = -this->lane_width_;
            } else {
                throw;
            }
            // 确定横向最终期望状态(横向速度)
            expected_end_vel.at(frame)[1] = 0.0;
            // 确定纵向最终期望状态(纵向位置)
            // 首先判断voxel中，前方是否存在车辆
            double start_time = this->time_stamps_.at(frame);
            double end_time = this->time_stamps_.at(frame + 1);
            std::pair<int, SemanticVehicle> front_vehicle;
            double vehicle_min_s_value = std::numeric_limits<double>::max();
            bool find_front = false;
            for (auto semantic_vehicle: surr_semantic_vehicle_set.semantic_vehicles) {
                if (current_voxel_node.voxel_.behavior_ == LateralBehavior::kLaneKeeping) {
                    if (semantic_vehicle.second.nearest_lane_id != ego_lane_id) {
                        continue;
                    }
                } else if (current_voxel_node.voxel_.behavior_ == LateralBehavior::kLaneChangeLeft) {
                    if (semantic_vehicle.second.nearest_lane_id != left_lane_id) {
                        continue;
                    }
                } else if (current_voxel_node.voxel_.behavior_ == LateralBehavior::kLaneChangeRight) {
                    if (semantic_vehicle.second.nearest_lane_id != right_lane_id) {
                        continue;
                    }
                } else {
                    throw;
                }
                // 计算障碍物车辆的移动距离
                double vehicle_min_s = semantic_vehicle.second.arc_len_onlane + semantic_vehicle.second.vehicle.state().velocity * start_time;
                // 增加障碍物车辆长度和自身车辆的长度
                vehicle_min_s -= semantic_vehicle.second.vehicle.param().rear_suspension() + semantic_ego_vehicle.vehicle.param().front_suspension() + semantic_ego_vehicle.vehicle.param().wheel_base();
                if (vehicle_min_s + kEPS >= current_voxel_node.voxel_.s_center_ + 0.5 * current_voxel_node.voxel_.s_len_) {
                    if (vehicle_min_s < vehicle_min_s_value) {
                        vehicle_min_s_value = vehicle_min_s;
                        front_vehicle = semantic_vehicle;
                        find_front = true;
                    }
                }
            }
            // 判断是否找到前方车
            if (!find_front) {
                // 前方无车, 设置为最大距离
                std::cout << "no vehicle in front" << std::endl;
                expected_end_pos.at(frame)[0] = current_voxel_node.voxel_.s_center_ + 0.5 * current_voxel_node.voxel_.s_len_;
                expected_end_vel.at(frame)[0] = std::min(semantic_ego_vehicle.vehicle.state().velocity + semantic_ego_vehicle.vehicle.param().max_longitudinal_acc() * end_time, this->max_longitude_vel_);
            } else {
                // 判断最后一个voxel中后方是否存在车辆
                std::pair<int, SemanticVehicle> rear_vehicle;
                double vehicle_max_s_value = - std::numeric_limits<double>::max();
                bool find_rear = false;
                for (auto semantic_vehicle: surr_semantic_vehicle_set.semantic_vehicles) {
                    if (semantic_vehicle.second.nearest_lane_id != front_vehicle.second.nearest_lane_id) {
                        continue;
                    }
                    // 计算障碍物车辆的移动距离
                    double vehicle_max_s = semantic_vehicle.second.arc_len_onlane + semantic_vehicle.second.vehicle.state().velocity * end_time;
                    // 增加障碍物车辆长度和自身车辆的长度
                    vehicle_max_s += semantic_ego_vehicle.vehicle.param().rear_suspension() + semantic_vehicle.second.vehicle.param().front_suspension() + semantic_vehicle.second.vehicle.param().wheel_base();
                    if (vehicle_max_s < vehicle_min_s_value + (end_time - start_time) * front_vehicle.second.vehicle.state().velocity) {
                        if (vehicle_max_s > vehicle_max_s_value) {
                            vehicle_max_s_value = vehicle_max_s;
                            rear_vehicle = semantic_vehicle;
                            find_rear = true;
                        }
                    }
                }
                if (!find_rear) {
                    // 前方有车，后方无车
                    std::cout << "no vehicle in rear" << std::endl;
                    std::cout << "front vehicle id: " << front_vehicle.first << std::endl;
                    std::cout << "vehicle_min_s_value: " << vehicle_min_s_value << ", vf: " << front_vehicle.second.vehicle.state().velocity << ", vs: " << semantic_ego_vehicle.vehicle.state().velocity << std::endl;
                    std::cout << "voxel range: " << current_voxel_node.voxel_.s_center_ - 0.5 * current_voxel_node.voxel_.s_len_ << " to " << current_voxel_node.voxel_.s_center_ + 0.5 * current_voxel_node.voxel_.s_len_ << std::endl;
                    // 根据前方车辆速度和本车速度确定期望位置
                    expected_end_pos.at(frame)[0] = vehicle_min_s_value + std::pow(front_vehicle.second.vehicle.state().velocity, 2.0) / (2.0 * front_vehicle.second.vehicle.param().max_lateral_acc()) - this->response_time_ * semantic_ego_vehicle.vehicle.state().velocity - std::pow(semantic_ego_vehicle.vehicle.state().velocity, 2.0) / (2.0 * semantic_ego_vehicle.vehicle.param().max_lateral_acc());
                    // 确定纵向最终期望状态(纵向速度)
                    expected_end_vel.at(frame)[0] = front_vehicle.second.vehicle.state().velocity;
                } else {
                    // 前后都有车
                    assert(front_vehicle.first != rear_vehicle.first);
                    std::cout << "front vehicle id: " << front_vehicle.first << ", rear vehicle id: " << rear_vehicle.first << std::endl;
                    std::cout << "vehicle_min_s_value: " << vehicle_min_s_value << ", vf: " << front_vehicle.second.vehicle.state().velocity << ", vs: " << semantic_ego_vehicle.vehicle.state().velocity << std::endl;
                    std::cout << "vehicle_max_s_value: " << vehicle_max_s_value << ", vr: " << rear_vehicle.second.vehicle.state().velocity << ", vs: " << semantic_ego_vehicle.vehicle.state().velocity << std::endl;
                    std::cout << "voxel range: " << current_voxel_node.voxel_.s_center_ - 0.5 * current_voxel_node.voxel_.s_len_ << " to " << current_voxel_node.voxel_.s_center_ + 0.5 * current_voxel_node.voxel_.s_len_ << std::endl;
                    // 根据前方车辆速度和本车速度确定期望位置
                    double front_expected_pos = vehicle_min_s_value + std::pow(front_vehicle.second.vehicle.state().velocity, 2.0) / (2.0 * front_vehicle.second.vehicle.param().max_lateral_acc()) - this->response_time_ * semantic_ego_vehicle.vehicle.state().velocity - std::pow(semantic_ego_vehicle.vehicle.state().velocity, 2.0) / (2.0 * semantic_ego_vehicle.vehicle.param().max_lateral_acc());
                    double rear_expected_pos = vehicle_max_s_value + rear_vehicle.second.vehicle.state().velocity * (this->response_time_ + end_time - start_time) + std::pow(rear_vehicle.second.vehicle.state().velocity, 2.0) / (2.0 * rear_vehicle.second.vehicle.param().max_lateral_acc()) - std::pow(semantic_ego_vehicle.vehicle.state().velocity, 2.0) / (2.0 * semantic_ego_vehicle.vehicle.param().max_lateral_acc());
                    std::cout << "front_expected_pos: " << front_expected_pos << ", rear_expected_pos: " << rear_expected_pos << std::endl;
                    if (rear_expected_pos <= front_expected_pos) {
                        expected_end_pos.at(frame)[0] = front_expected_pos;
                    } else {
                        expected_end_pos.at(frame)[0] = 0.5 * front_expected_pos + 0.5 * rear_expected_pos;
                    }
                    // 确定纵向最终期望状态(纵向速度)
                    expected_end_vel.at(frame)[0] = front_vehicle.second.vehicle.state().velocity;
                }
            }
        }
        return kSuccess;
    }

    // 进行轨迹生成
    template <int N_DEG, int N_DIM>
    ErrorType trajectoryGeneration(const SemanticVehicle &semantic_ego_vehicle, const SemanticVehicleSet &surr_semantic_vehicle_set, const SemanticLaneSet &semantic_lane_set, const std::vector<std::pair<std::vector<Node>, double>> &voxel_sequences_with_cost, std::vector<std::pair<LateralBehavior, FrenetBezierTrajectory<N_DEG,N_DIM>>> &frenet_bezier_splines) const {
        if (N_DIM != 2) {
            throw;
        }
        // 得到frenet变换
        StateTransformer stf(semantic_ego_vehicle.lane);
        FrenetState init_ego_vehicle_fs;
        if (stf.GetFrenetStateFromState(semantic_ego_vehicle.vehicle.state(), &init_ego_vehicle_fs) != kSuccess) {
            throw;
        }
        // 得到初始约束
        vec_E<Vecf<N_DIM>> start_constraints;
        start_constraints.push_back(Vecf<N_DIM>(init_ego_vehicle_fs.vec_s[0], init_ego_vehicle_fs.vec_dt[0]));
        start_constraints.push_back(Vecf<N_DIM>(init_ego_vehicle_fs.vec_s[1], init_ego_vehicle_fs.vec_dt[1]));
        start_constraints.push_back(Vecf<N_DIM>(init_ego_vehicle_fs.vec_s[2], init_ego_vehicle_fs.vec_dt[2]));
        for (auto voxel_sequence_with_cost: voxel_sequences_with_cost) {
            std::vector<Node> voxel_sequence = voxel_sequence_with_cost.first;
            while (true) {
                std::cout << "************* voxel sequence: *************" << std::endl;
                for (auto voxel_node: voxel_sequence) {
                    std::cout << "voxel: s(" << voxel_node.voxel_.s_center_ - 0.5 * voxel_node.voxel_.s_len_ << "," << voxel_node.voxel_.s_center_ + 0.5 * voxel_node.voxel_.s_len_ << ") d(" << voxel_node.voxel_.d_center_ - 0.5 * voxel_node.voxel_.d_len_ << "," << voxel_node.voxel_.d_center_ + 0.5 * voxel_node.voxel_.d_len_ << ") t(" << voxel_node.voxel_.t_center_ - 0.5 * voxel_node.voxel_.t_len_ << "," << voxel_node.voxel_.t_center_ + 0.5 * voxel_node.voxel_.t_len_ << ") behavior->" << static_cast<int>(voxel_node.voxel_.behavior_) << std::endl;
                }
                // 得到期望终点状态
                vec_E<Vecf<N_DIM>> expected_end_pos, expected_end_vel;
                if (this->generateExpectedState(semantic_ego_vehicle, surr_semantic_vehicle_set, semantic_lane_set, voxel_sequence, expected_end_pos, expected_end_vel) != kSuccess) {
                    throw;
                }
                std::cout << "start contraints:\n" << start_constraints[0] << "; " << start_constraints[1] << "; " << start_constraints[2] << std::endl;
                // 进行贝塞尔轨迹生成
                BezierSpline<N_DEG, N_DIM> bezier_spline;
                if (this->trajectoryGenerationWithVoxels<N_DEG, N_DIM>(semantic_ego_vehicle, voxel_sequence, start_constraints, expected_end_pos, expected_end_vel, bezier_spline) == kSuccess) {
                    // 调试信息
                    Vec2f start_0d, start_1d, start_2d, end_0d, end_1d, inter_0d, inter_1d, inter_2d;
                    bezier_spline.evaluate(bezier_spline.begin(), 0, &start_0d);
                    bezier_spline.evaluate(bezier_spline.begin(), 1, &start_1d);
                    bezier_spline.evaluate(bezier_spline.begin(), 2, &start_2d);
                    bezier_spline.evaluate(bezier_spline.end(), 0, &end_0d);
                    bezier_spline.evaluate(bezier_spline.end(), 1, &end_1d);
                    bezier_spline.evaluate(0.2, 0, &inter_0d);
                    bezier_spline.evaluate(0.2, 1, &inter_1d);
                    bezier_spline.evaluate(0.2, 2, &inter_2d);
                    std::cout << "start: " << start_0d << "; " << start_1d << "; " << start_2d << std::endl;
                    std::cout << "next: " << inter_0d << "; " << inter_1d << "; " << inter_2d << std::endl;
                    std::cout << "end: " << end_0d << "; " << end_1d << std::endl;
                    // 进行保存
                    FrenetBezierTrajectory<N_DEG, N_DIM> frenet_bezier_spline(bezier_spline, stf);
                    // 进行轨迹验证
                    if (this->validateTrajectory(frenet_bezier_spline, semantic_ego_vehicle.vehicle.state(), semantic_ego_vehicle.vehicle.param()) == kSuccess) {
                        frenet_bezier_splines.push_back(std::make_pair(voxel_sequence.back().voxel_.behavior_, frenet_bezier_spline));
                        break;
                    }
                }
                // 如果优化或验证没能成功
                auto last_voxel = voxel_sequence.back();
                voxel_sequence.pop_back();
                // 判断长度
                if (voxel_sequence.size() < this->min_valid_num_) {
                    break;
                }
                if (voxel_sequence.back().voxel_.behavior_ != last_voxel.voxel_.behavior_) {
                    break;
                }
            }
        }
        // 判断是否生成成功
        if (frenet_bezier_splines.size() == 0) {
            // 生成轨迹失败
            return kWrongStatus;
        }
        // 进行轨迹可视化
        {
            // 删除之前的可视化
            visualization_msgs::MarkerArray delete_marker_array;
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "odom";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.action = visualization_msgs::Marker::DELETEALL;
            delete_marker.id = 0;
            delete_marker_array.markers.push_back(delete_marker);
            this->debug_trajectories_vis_.publish(delete_marker_array);
            // 进行可视化
            int id = 0;
            visualization_msgs::MarkerArray marker_array;
            for (auto frenet_bezier_spline: frenet_bezier_splines) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = ros::Time::now();
                marker.id = ++id;
                marker.action = visualization_msgs::Marker::ADD;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                marker.scale.x = 0.12;
                std::vector<double> samples;
                GetRangeVector<double>(frenet_bezier_spline.second.begin(), frenet_bezier_spline.second.end(), 0.1, true, &samples);
                for (auto sample: samples) {
                    geometry_msgs::Point point;
                    State state;
                    frenet_bezier_spline.second.GetState(sample, &state);
                    point.x = state.vec_position(0);
                    point.y = state.vec_position(1);
                    marker.points.push_back(point);
                }
                marker_array.markers.push_back(marker);
            }
            this->debug_trajectories_vis_.publish(marker_array);
        }
        return kSuccess;
    }

    // 进行轨迹生成
    template <int N_DEG, int N_DIM>
    ErrorType trajectoryGenerationWithVoxels(const SemanticVehicle &semantic_ego_vehicle, const std::vector<Node> &voxel_sequence, const vec_E<Vecf<N_DIM>>& start_constraints, const vec_E<Vecf<N_DIM>>& expected_end_pos, const vec_E<Vecf<N_DIM>>& expected_end_vel, BezierSpline<N_DEG, N_DIM> &bezier_spline) const {
        if (N_DIM != 2) {
            throw;
        }

        int num_segments = static_cast<int>(voxel_sequence.size());
        int num_order = N_DEG + 1;
        int derivative_degree = 3;
        
        // 目标函数 0.5 * x^T * Q * x + c * x
        int total_num_vals = N_DIM * num_segments * num_order;
        // 初始化参数矩阵
        Eigen::SparseMatrix<double, Eigen::RowMajor> Q(total_num_vals, total_num_vals);
        Q.reserve(Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
        Eigen::VectorXd c;
        c.resize(total_num_vals);
        c.setZero();

        // 进行参数矩阵Q1和向量c1构建(jerk最小化)
        // 初始化参数矩阵
        Eigen::SparseMatrix<double, Eigen::RowMajor> Q1(total_num_vals, total_num_vals);
        Q1.reserve(Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
        Eigen::VectorXd c1;
        c1.resize(total_num_vals);
        c1.setZero();
        {
            int idx, idy;
            double val;
            MatNf<N_DEG + 1> hessian = BezierUtils<N_DEG>::GetBezierHessianMat(derivative_degree);
            for (int n = 0; n < num_segments; n++) {
                double duration = voxel_sequence.at(n).voxel_.t_len_;
                for (int d = 0; d < N_DIM; d++) {
                    for (int j = 0; j < num_order; j++) {
                        for (int k = 0; k < num_order; k++) {
                            int idx = d * num_segments * num_order + n * num_order + j;
                            int idy = d * num_segments * num_order + n * num_order + k;
                            double val = hessian(j, k) / pow(duration, 2 * derivative_degree - 3);
                            Q1.insert(idx, idy) = val;
                        }
                    }
                }
            }
        }
        
        // 进行参数矩阵Q2和向量c2构建(末状态的位置优化)
        // 初始化参数矩阵
        Eigen::SparseMatrix<double, Eigen::RowMajor> Q2(total_num_vals, total_num_vals);
        Q2.reserve(Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
        Eigen::VectorXd c2;
        c2.resize(total_num_vals);
        c2.setZero();
        assert(expected_end_pos.size() == num_segments);
        {
            int idx, idy;
            double val;
            for (int n = 0; n < num_segments; n++) {
                double duration = voxel_sequence.at(n).voxel_.t_len_;
                for (int d = 0; d < N_DIM; d++) {
                    idx = d * num_segments * num_order + n * num_order + N_DEG;
                    idy = d * num_segments * num_order + n * num_order + N_DEG;
                    val = 1.0 * std::pow(duration, 2.0);
                    Q2.insert(idx, idy) = val;
                }
                for (int d = 0; d < N_DIM; d++) {
                    idx = d * num_segments * num_order + n * num_order + N_DEG;
                    val = 1.0 * duration;
                    c2[idx] = -2.0 * val * expected_end_pos[n][d];
                }
            }
        }

        // 进行参数矩阵Q3和向量c3构建(末状态的速度优化)
        Eigen::SparseMatrix<double, Eigen::RowMajor> Q3(total_num_vals, total_num_vals);
        Q3.reserve(Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
        Eigen::VectorXd c3;
        c3.resize(total_num_vals);
        c3.setZero();
        assert(expected_end_vel.size() == num_segments);
        {
            int idx, idy;
            double val;
            for (int n = 0; n < num_segments; n++) {
                for (int d = 0; d < N_DIM; d++) {
                    idx = d * num_segments * num_order + n * num_order + N_DEG - 1;
                    idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
                    val = N_DEG * N_DEG;
                    Q3.insert(idx, idy) = val;
                    idx = d * num_segments * num_order + n * num_order + N_DEG - 1;
                    idy = d * num_segments * num_order + n * num_order + N_DEG;
                    val = -1.0 * N_DEG * N_DEG;
                    Q3.insert(idx, idy) = val;
                    idx = d * num_segments * num_order + n * num_order + N_DEG;
                    idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
                    val = -1.0 * N_DEG * N_DEG;
                    Q3.insert(idx, idy) = val;
                    idx = d * num_segments * num_order + n * num_order + N_DEG;
                    idy = d * num_segments * num_order + n * num_order + N_DEG;
                    val = N_DEG * N_DEG;
                    Q3.insert(idx, idy) = val;
                }
                for (int d = 0; d < N_DIM; d++) {
                    idx = d * num_segments * num_order + n * num_order + N_DEG - 1;
                    val = N_DEG;
                    c3[idx] = 2.0 * val * expected_end_vel[n][d];
                    idx = d * num_segments * num_order + n * num_order + N_DEG;
                    val = N_DEG;
                    c3[idx] = -2.0 * val * expected_end_vel[n][d];
                }
            }
        }

        // 进行参数矩阵Q4和向量c4构建(横向速度平方的积分尽可能小)（让横向速度尽可能接近与常数，横向位置尽可能接近于均匀变化）
        // 初始化参数矩阵
        Eigen::SparseMatrix<double, Eigen::RowMajor> Q4(total_num_vals, total_num_vals);
        Q4.reserve(Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
        Eigen::VectorXd c4;
        c4.resize(total_num_vals);
        c4.setZero();
        {
            int idx, idy;
            double val;
            MatNf<N_DEG + 1> hessian = BezierUtils<N_DEG>::GetBezierHessianMat(1);
            int d = 1;
            for (int n = 0; n < num_segments; n++) {
                double duration = voxel_sequence.at(n).voxel_.t_len_;
                for (int j = 0; j < num_order; j++) {
                    for (int k = 0; k < num_order; k++) {
                        int idx = d * num_segments * num_order + n * num_order + j;
                        int idy = d * num_segments * num_order + n * num_order + k;
                        double val = hessian(j, k) * duration;
                        Q4.insert(idx, idy) = val;
                    }
                }
            }
        }

        // 进行参数矩阵Q5和向量c5构建(纵向加速度平方的积分尽可能小)（让纵向加速度尽可能接近与常数，横向速度尽可能接近于均匀变化）
        // 初始化参数矩阵
        Eigen::SparseMatrix<double, Eigen::RowMajor> Q5(total_num_vals, total_num_vals);
        Q5.reserve(Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
        Eigen::VectorXd c5;
        c5.resize(total_num_vals);
        c5.setZero();
        {
            int idx, idy;
            double val;
            MatNf<N_DEG + 1> hessian = BezierUtils<N_DEG>::GetBezierHessianMat(2);
            int d = 0;
            for (int n = 0; n < num_segments; n++) {
                double duration = voxel_sequence.at(n).voxel_.t_len_;
                for (int j = 0; j < num_order; j++) {
                    for (int k = 0; k < num_order; k++) {
                        int idx = d * num_segments * num_order + n * num_order + j;
                        int idy = d * num_segments * num_order + n * num_order + k;
                        double val = hessian(j, k) / duration;
                        Q5.insert(idx, idy) = val;
                    }
                }
            }
        }

        // 防止数值误差
        Eigen::SparseMatrix<double, Eigen::RowMajor> Err(total_num_vals, total_num_vals);
        Err.reserve(Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
        for (int i = 0; i < total_num_vals; i++) {
            Err.insert(i, i) = kEPS;
        }

        // 进行加权求和
        Q = 2 * (0.1 * Q1 + 10.0 * Q2 + 10.0 * Q3 + Q4 + Q5 + Err);
        c = (0.1 * c1 + 10.0 * c2 + 10.0 * c3 + c4 + c5);
        
        // 进行验证
        Eigen::MatrixXd dq_mat = Eigen::MatrixXd(Q);
        // 判断Q矩阵是否对称
        if (dq_mat == dq_mat.transpose()) {
            std::cout << "Q is symmetric" << std::endl;
        } else {
            std::cout << "Q is not symmetric" << std::endl;
        }
        // 判断Q矩阵是否正定
        bool is_psd = isPsd<Eigen::MatrixXd>(dq_mat);
        if (is_psd) {
            std::cout << "Q is psd" << std::endl;
        } else {
            std::cout << "Q is not psd" << std::endl;
        }
        
        // 等式约束 A * x = b
        int num_continuity = 3;  // continuity up to acc
        int num_connections = num_segments - 1;
        int num_continuity_constraints = N_DIM * num_connections * num_continuity;
        int num_start_eq_constraints = static_cast<int>(start_constraints.size()) * N_DIM;
        int total_num_eq_constraints = num_continuity_constraints + num_start_eq_constraints;
        // 初始化参数矩阵
        Eigen::SparseMatrix<double, Eigen::RowMajor> A(
        total_num_eq_constraints, N_DIM * num_segments * num_order);
        A.reserve(Eigen::VectorXi::Constant(total_num_eq_constraints, 2 * num_order));
        Eigen::VectorXd b;
        b.resize(total_num_eq_constraints);
        b.setZero();
        // 构建参数矩阵A
        {
            int idx, idy;
            double val;
            // 连续性约束
            for (int n = 0; n < num_connections; n++) {
                double duration_l = voxel_sequence.at(n).voxel_.t_len_;
                double duration_r = voxel_sequence.at(n + 1).voxel_.t_len_;
                for (int m = 0; m < num_continuity; m++) {
                    double scale_l = pow(duration_l, 1 - m);
                    double scale_r = pow(duration_r, 1 - m);
                    for (int d = 0; d < N_DIM; d++) {
                        idx = d * num_connections * num_continuity + n * num_continuity + m;
                        if (m == 0) {
                            // ~ position end
                            idy = d * num_segments * num_order + n * num_order + N_DEG;
                            val = 1.0 * scale_l;
                            A.insert(idx, idy) = val;
                            // ~ position begin
                            idy = d * num_segments * num_order + (n + 1) * num_order + 0;
                            val = 1.0 * scale_r;
                            A.insert(idx, idy) = -val;
                        } else if (m == 1) {
                            // ~ velocity end
                            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
                            val = -1.0 * scale_l;
                            A.insert(idx, idy) = val;
                            idy = d * num_segments * num_order + n * num_order + N_DEG;
                            val = 1.0 * scale_l;
                            A.insert(idx, idy) = val;
                            // ~ velocity begin
                            idy = d * num_segments * num_order + (n + 1) * num_order;
                            val = -1.0 * scale_r;
                            A.insert(idx, idy) = -val;
                            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
                            val = 1.0 * scale_r;
                            A.insert(idx, idy) = -val;
                        } else if (m == 2) {
                            // ~ acceleration end
                            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
                            val = 1.0 * scale_l;
                            A.insert(idx, idy) = val;
                            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
                            val = -2.0 * scale_l;
                            A.insert(idx, idy) = val;
                            idy = d * num_segments * num_order + n * num_order + N_DEG;
                            val = 1.0 * scale_l;
                            A.insert(idx, idy) = val;
                            // ~ acceleration begin
                            idy = d * num_segments * num_order + (n + 1) * num_order;
                            val = 1.0 * scale_r;
                            A.insert(idx, idy) = -val;
                            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
                            val = -2.0 * scale_r;
                            A.insert(idx, idy) = -val;
                            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
                            val = 1.0 * scale_r;
                            A.insert(idx, idy) = -val;
                        } else if (m == 3) {
                            // ~ jerk end
                            idy = d * num_segments * num_order + n * num_order + N_DEG - 3;
                            val = -1.0 * scale_l;
                            A.insert(idx, idy) = val;
                            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
                            val = 3.0 * scale_l;
                            A.insert(idx, idy) = val;
                            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
                            val = -3.0 * scale_l;
                            A.insert(idx, idy) = val;
                            idy = d * num_segments * num_order + n * num_order + N_DEG;
                            val = 1.0 * scale_l;
                            A.insert(idx, idy) = val;
                            // ~ jerk begin
                            idy = d * num_segments * num_order + (n + 1) * num_order;
                            val = -1.0 * scale_r;
                            A.insert(idx, idy) = -val;
                            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
                            val = 3.0 * scale_r;
                            A.insert(idx, idy) = -val;
                            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
                            val = -3.0 * scale_r;
                            A.insert(idx, idy) = -val;
                            idy = d * num_segments * num_order + (n + 1) * num_order + 3;
                            val = 1.0 * scale_r;
                            A.insert(idx, idy) = -val;
                        }
                    }
                }
            }

            // 初始状态条件约束
            {
                int num_order_constraint_start = static_cast<int>(start_constraints.size());
                double duration = voxel_sequence.at(0).voxel_.t_len_;
                double scale;
                int n = 0;
                for (int j = 0; j < num_order_constraint_start; j++) {
                    scale = pow(duration, 1 - j);
                    for (int d = 0; d < N_DIM; d++) {
                        idx = num_continuity_constraints + d * num_order_constraint_start + j;
                        if (j == 0) {
                            idy = d * num_segments * num_order + n * num_order + 0;
                            val = 1.0 * scale;
                            A.insert(idx, idy) = val;

                            b[idx] = start_constraints[j][d];
                            // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
                        } else if (j == 1) {
                            idy = d * num_segments * num_order + n * num_order + 0;
                            val = -1.0 * N_DEG * scale;
                            A.insert(idx, idy) = val;
                            idy = d * num_segments * num_order + n * num_order + 1;
                            val = 1.0 * N_DEG * scale;
                            A.insert(idx, idy) = val;

                            b[idx] = start_constraints[j][d];
                            // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
                        } else if (j == 2) {
                            idy = d * num_segments * num_order + n * num_order + 0;
                            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
                            A.insert(idx, idy) = val;

                            idy = d * num_segments * num_order + n * num_order + 1;
                            val = -2.0 * N_DEG * (N_DEG - 1) * scale;
                            A.insert(idx, idy) = val;

                            idy = d * num_segments * num_order + n * num_order + 2;
                            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
                            A.insert(idx, idy) = val;

                            b[idx] = start_constraints[j][d];
                            // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
                        }
                    }
                }
            }
        }

        // 不等式约束 lbd <= C * x <= ubd
        int total_num_ineq = 0;
        for (int i = 0; i < num_segments; i++) {
            total_num_ineq += N_DIM * num_order;
            total_num_ineq += N_DIM * (num_order - 1);
            total_num_ineq += N_DIM * (num_order - 2);
            total_num_ineq += N_DIM * (num_order - 3);
        }
        Eigen::VectorXd lbd;
        Eigen::VectorXd ubd;
        Eigen::SparseMatrix<double, Eigen::RowMajor> C(total_num_ineq, total_num_vals);
        C.reserve(Eigen::VectorXi::Constant(total_num_ineq, 3));
        lbd.setZero(total_num_ineq);
        ubd.setZero(total_num_ineq);
        {
            int accu_num_ineq = 0;
            int idx, idy;
            double val;
            for (int n = 0; n < num_segments; n++) {
                double duration = voxel_sequence.at(n).voxel_.t_len_;
                double scale;
                for (int d = 0; d < N_DIM; d++) {
                    // ~ enforce position bounds
                    scale = pow(duration, 1 - 0);
                    for (int j = 0; j < num_order; j++) {
                        idx = accu_num_ineq;
                        idy = d * num_segments * num_order + n * num_order + j;
                        val = scale;
                        C.insert(idx, idy) = val;
                        if (d == 0) {
                            lbd[idx] = voxel_sequence.at(n).voxel_.s_center_ - 0.5 * voxel_sequence.at(n).voxel_.s_len_;
                            ubd[idx] = voxel_sequence.at(n).voxel_.s_center_ + 0.5 * voxel_sequence.at(n).voxel_.s_len_ ;
                        } else {
                            lbd[idx] = voxel_sequence.at(n).voxel_.d_center_ - 0.5 * voxel_sequence.at(n).voxel_.d_len_;
                            ubd[idx] = voxel_sequence.at(n).voxel_.d_center_ + 0.5 * voxel_sequence.at(n).voxel_.d_len_;
                        }
                        accu_num_ineq++;
                    }
                    // ~ enforce velocity bounds
                    scale = pow(duration, 1 - 1);
                    for (int j = 0; j < num_order - 1; j++) {
                        idx = accu_num_ineq;
                        idy = d * num_segments * num_order + n * num_order + j;
                        val = -N_DEG * scale;
                        C.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + (j + 1);
                        val = N_DEG * scale;
                        C.insert(idx, idy) = val;
                        if (d == 0) {
                            lbd[idx] = 0.0;
                            ubd[idx] = this->max_longitude_vel_;
                        } else {
                            lbd[idx] = -semantic_ego_vehicle.vehicle.param().max_lateral_vel();
                            ubd[idx] = semantic_ego_vehicle.vehicle.param().max_lateral_vel();
                        }
                        accu_num_ineq++;
                    }
                    // ~ enforce acceleration bounds
                    scale = pow(duration, 1 - 2);
                    for (int j = 0; j < num_order - 2; j++) {
                        idx = accu_num_ineq;
                        idy = d * num_segments * num_order + n * num_order + j;
                        val = N_DEG * (N_DEG - 1) * scale;
                        C.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + (j + 1);
                        val = -2.0 * N_DEG * (N_DEG - 1) * scale;
                        C.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + (j + 2);
                        val = N_DEG * (N_DEG - 1) * scale;
                        C.insert(idx, idy) = val;
                        if (d == 0) {
                            lbd[idx] = - semantic_ego_vehicle.vehicle.param().max_longitudinal_acc();
                            ubd[idx] = semantic_ego_vehicle.vehicle.param().max_longitudinal_acc();
                        } else {
                            lbd[idx] = - semantic_ego_vehicle.vehicle.param().max_lateral_acc();
                            ubd[idx] = semantic_ego_vehicle.vehicle.param().max_lateral_acc();
                        }
                        accu_num_ineq++;
                    }
                    // ~ enforce jerk bounds
                    scale = pow(duration, 1 - 3);
                    for (int j = 0; j < num_order - 3; j++) {
                        idx = accu_num_ineq;
                        idy = d * num_segments * num_order + n * num_order + j;
                        val = - N_DEG * (N_DEG - 1) * (N_DEG - 2) * scale;
                        C.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + (j + 1);
                        val = 3.0 * N_DEG * (N_DEG - 1) * (N_DEG - 2) * scale;
                        C.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + (j + 2);
                        val = -3.0 * N_DEG * (N_DEG - 1) * (N_DEG - 2) * scale;
                        C.insert(idx, idy) = val;
                        idy = d * num_segments * num_order + n * num_order + (j + 3);
                        val = N_DEG * (N_DEG - 1) * (N_DEG - 2) * scale;
                        C.insert(idx, idy) = val;
                        if (d == 0) {
                            lbd[idx] = - semantic_ego_vehicle.vehicle.param().max_longitudinal_jerk();
                            ubd[idx] = semantic_ego_vehicle.vehicle.param().max_longitudinal_jerk();
                        } else {
                            lbd[idx] = - semantic_ego_vehicle.vehicle.param().max_lateral_jerk();
                            ubd[idx] = semantic_ego_vehicle.vehicle.param().max_lateral_jerk();
                        }
                        accu_num_ineq++;
                    }
                }
            }
        }

        // dummy constraints l <= x <= u
        Eigen::VectorXd u = std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(total_num_vals);
        Eigen::VectorXd l = (-u.array()).matrix();

        // 进行优化
        Eigen::VectorXd x;
        x.setZero(total_num_vals);
        if (!OoQpItf::solve(Q, c, A, b, C, lbd, ubd, l, u, x, true, false)) {
            printf("trajectory generation solver failed.\n");
            return kWrongStatus;
        }
        // std::cout << "result x: " << x.transpose() << std::endl;
        std::cout << "term 1: " << x.transpose() * Q1 * x + c1.transpose() * x << std::endl;
        std::cout << "term 2: " << x.transpose() * Q2 * x + c2.transpose() * x << std::endl;
        std::cout << "term 3: " << x.transpose() * Q3 * x + c3.transpose() * x << std::endl;
        std::cout << "term 4: " << x.transpose() * Q4 * x + c4.transpose() * x << std::endl;
        std::cout << "term 5: " << x.transpose() * Q5 * x + c5.transpose() * x << std::endl;

        // 进行贝塞尔曲线构建
        std::vector<double> vec_domain;
        vec_domain.push_back(voxel_sequence.front().voxel_.t_center_ - 0.5 * voxel_sequence.front().voxel_.t_len_);
        for (int n = 0; n < num_segments; n++) {
            vec_domain.push_back(voxel_sequence.at(n).voxel_.t_center_ + 0.5 * voxel_sequence.at(n).voxel_.t_len_);
        }
        bezier_spline.set_vec_domain(vec_domain);
        for (int n = 0; n < num_segments; n++) {
            for (int j = 0; j < num_order; j++) {
                Vecf<N_DIM> coeff;
                for (int d = 0; d < N_DIM; d++) {
                    coeff[d] = x[d * num_segments * num_order + n * num_order + j];
                }
                bezier_spline.set_coeff(n, j, coeff);
            }
        }
        return kSuccess;
    }

    // 对轨迹进行验证
    ErrorType validateTrajectory(const FrenetTrajectory& traj, const State &initial_state, const VehicleParam &param) const {
        std::vector<double> t_vec_xy;
        GetRangeVector<double>(traj.begin(), traj.end(), 0.1, true, &t_vec_xy);
        State state;
        // * check init state
        if (traj.GetState(traj.begin(), &state) != kSuccess) {
            std::cout << "[Ssc][Validate]State evaluation error" << std::endl;
            return kWrongStatus;
        }

        if ((state.vec_position - initial_state.vec_position).norm() > kBigEPS) {
            std::cout << "[Ssc][Validate]Init position miss match" << std::endl;
            return kWrongStatus;
        }

        if (fabs(state.velocity - initial_state.velocity) > kBigEPS) {
            std::cout << "[Ssc][Validate]Init vel miss match" << std::endl;
            return kWrongStatus;
        }
        // * check end state
        if (traj.GetState(traj.end(), &state) != kSuccess) {
            std::cout << "[Ssc][Validate]End state eval error " << traj.end() << std::endl;
            return kWrongStatus;
        }

        for (const auto t : t_vec_xy) {
            if (traj.GetState(t, &state) != kSuccess) {
                std::cout << "[Ssc][Validate]State eval error" << std::endl;
                return kWrongStatus;
            }
            if (fabs(state.curvature) > param.max_curvature()) {
                std::cout << "[Ssc][Validate]initial_state velocity "<< initial_state.velocity << " Curvature " << state.curvature << " invalid." << std::endl;
                return kWrongStatus;
            }
        }
        return kSuccess;
    }

    // 进行栅格序列修正
    ErrorType voxelRevise(const std::vector<std::vector<Voxel>> &voxels, const SemanticVehicle &semantic_ego_vehicle, std::vector<std::pair<std::vector<Node>, double>> &voxel_sequences_with_cost) const {
        // 进行纵向修正
        for (auto &voxel_sequence_with_cost: voxel_sequences_with_cost) {
            if (voxel_sequence_with_cost.first.back().voxel_.behavior_ != LateralBehavior::kLaneKeeping) {
                for (int i = 0; i < voxel_sequence_with_cost.first.size() - 1; i++) {
                    if (voxel_sequence_with_cost.first.at(i).voxel_.behavior_ != voxel_sequence_with_cost.first.at(i + 1).voxel_.behavior_) {
                        assert(voxel_sequence_with_cost.first.at(i).voxel_.behavior_ == LateralBehavior::kLaneKeeping);
                        // 找到中相同行为的节点
                        std::vector<Voxel> same_behavior_voxels_in_last_graph_layer;
                        for (auto voxel: voxels.at(i)) {
                            if (voxel_sequence_with_cost.first.at(i + 1).voxel_.behavior_ == voxel.behavior_) {
                                same_behavior_voxels_in_last_graph_layer.push_back(voxel);
                            }
                        }
                        if (same_behavior_voxels_in_last_graph_layer.size() == 0) {
                            throw;
                        }
                        // 找到本层中道路保持的其他节点
                        std::vector<Voxel> lane_keeping_voxels_in_current_layer;
                        for (auto voxel: voxels.at(i + 1)) {
                            if (voxel.behavior_ == LateralBehavior::kLaneKeeping) {
                                lane_keeping_voxels_in_current_layer.push_back(voxel);
                            }
                        }
                        if (lane_keeping_voxels_in_current_layer.size() == 0) {
                            throw;
                        }
                        // 找到最大重叠
                        double s_intersection_len = 0.0;
                        Voxel chosed_last_layer_voxel, chosed_current_layer_voxel;
                        for (auto last_layer_voxel: same_behavior_voxels_in_last_graph_layer) {
                            for (auto current_layer_voxel: lane_keeping_voxels_in_current_layer) {
                                double min_boundary = std::max({voxel_sequence_with_cost.first.at(i + 1).voxel_.s_center_ - 0.5 * voxel_sequence_with_cost.first.at(i + 1).voxel_.s_len_, current_layer_voxel.s_center_ - 0.5 * current_layer_voxel.s_len_, voxel_sequence_with_cost.first.at(i).voxel_.s_center_ - 0.5 * voxel_sequence_with_cost.first.at(i).voxel_.s_len_, last_layer_voxel.s_center_ - 0.5 * last_layer_voxel.s_len_});
                                double max_boundary = std::min({voxel_sequence_with_cost.first.at(i+ 1).voxel_.s_center_ + 0.5 * voxel_sequence_with_cost.first.at(i + 1).voxel_.s_len_, current_layer_voxel.s_center_ + 0.5 * current_layer_voxel.s_len_, voxel_sequence_with_cost.first.at(i).voxel_.s_center_ + 0.5 * voxel_sequence_with_cost.first.at(i).voxel_.s_len_, last_layer_voxel.s_center_ + 0.5 * last_layer_voxel.s_len_});
                                if (max_boundary - min_boundary > s_intersection_len) {
                                    s_intersection_len = max_boundary - min_boundary;
                                    chosed_last_layer_voxel = last_layer_voxel;
                                    chosed_current_layer_voxel = current_layer_voxel;
                                }
                            }
                        }
                        if (s_intersection_len <= 0) {
                            throw;
                        }
                        // 进行voxel更新
                        double min_boundary, max_boundary;
                        min_boundary = std::max(voxel_sequence_with_cost.first.at(i).voxel_.s_center_ - 0.5 * voxel_sequence_with_cost.first.at(i).voxel_.s_len_, chosed_last_layer_voxel.s_center_ - 0.5 * chosed_last_layer_voxel.s_len_);
                        max_boundary = std::min(voxel_sequence_with_cost.first.at(i).voxel_.s_center_ + 0.5 * voxel_sequence_with_cost.first.at(i).voxel_.s_len_, chosed_last_layer_voxel.s_center_ + 0.5 * chosed_last_layer_voxel.s_len_);
                        assert(max_boundary > min_boundary);
                        voxel_sequence_with_cost.first.at(i).voxel_.s_len_ = max_boundary - min_boundary;
                        voxel_sequence_with_cost.first.at(i).voxel_.s_center_ = 0.5 * (max_boundary + min_boundary);

                        min_boundary = std::max(voxel_sequence_with_cost.first.at(i + 1).voxel_.s_center_ - 0.5 * voxel_sequence_with_cost.first.at(i + 1).voxel_.s_len_, chosed_current_layer_voxel.s_center_ - 0.5 * chosed_current_layer_voxel.s_len_);
                        max_boundary = std::min(voxel_sequence_with_cost.first.at(i + 1).voxel_.s_center_ + 0.5 * voxel_sequence_with_cost.first.at(i + 1).voxel_.s_len_, chosed_current_layer_voxel.s_center_ + 0.5 * chosed_current_layer_voxel.s_len_);
                        assert(max_boundary > min_boundary);
                        voxel_sequence_with_cost.first.at(i + 1).voxel_.s_len_ = max_boundary - min_boundary;
                        voxel_sequence_with_cost.first.at(i + 1).voxel_.s_center_ = 0.5 * (max_boundary + min_boundary);
                        break;
                    }
                }
            }
        }
        // 进行横向栅格修正
        for (auto &voxel_sequence_with_cost: voxel_sequences_with_cost) {
            if (voxel_sequence_with_cost.first.back().voxel_.behavior_ != LateralBehavior::kLaneKeeping) {
                for (int i = 0; i < voxel_sequence_with_cost.first.size() - 1; i++) {
                    if (voxel_sequence_with_cost.first.at(i).voxel_.behavior_ != voxel_sequence_with_cost.first.at(i + 1).voxel_.behavior_) {
                        if (voxel_sequence_with_cost.first.at(i).voxel_.behavior_ != LateralBehavior::kLaneKeeping) {
                            throw;
                        }
                        if (voxel_sequence_with_cost.first.at(i + 1).voxel_.behavior_ == LateralBehavior::kLaneChangeLeft) {
                            double left_d, right_d;
                            left_d = std::max(voxel_sequence_with_cost.first.at(i).voxel_.d_center_ + 0.5 * voxel_sequence_with_cost.first.at(i).voxel_.d_len_, this->lane_width_);
                            right_d = voxel_sequence_with_cost.first.at(i).voxel_.d_center_ - 0.5 * voxel_sequence_with_cost.first.at(i).voxel_.d_len_;
                            voxel_sequence_with_cost.first.at(i).voxel_.d_center_ = 0.5 * (left_d + right_d);
                            voxel_sequence_with_cost.first.at(i).voxel_.d_len_ = left_d - right_d;
                            
                            left_d = voxel_sequence_with_cost.first.at(i + 1).voxel_.d_center_ + 0.5 * voxel_sequence_with_cost.first.at(i + 1).voxel_.d_len_;
                            right_d = std::min(voxel_sequence_with_cost.first.at(i + 1).voxel_.d_center_ - 0.5 * voxel_sequence_with_cost.first.at(i + 1).voxel_.d_len_, 0.0);
                            voxel_sequence_with_cost.first.at(i + 1).voxel_.d_center_ = 0.5 * (left_d + right_d);
                            voxel_sequence_with_cost.first.at(i + 1).voxel_.d_len_ = left_d - right_d;
                        } else if (voxel_sequence_with_cost.first.at(i + 1).voxel_.behavior_ == LateralBehavior::kLaneChangeRight) {
                            double left_d, right_d;
                            left_d = voxel_sequence_with_cost.first.at(i).voxel_.d_center_ + 0.5 * voxel_sequence_with_cost.first.at(i).voxel_.d_len_;
                            right_d = std::min(voxel_sequence_with_cost.first.at(i).voxel_.d_center_ - 0.5 * voxel_sequence_with_cost.first.at(i).voxel_.d_len_, -this->lane_width_);
                            voxel_sequence_with_cost.first.at(i).voxel_.d_center_ = 0.5 * (left_d + right_d);
                            voxel_sequence_with_cost.first.at(i).voxel_.d_len_ = left_d - right_d;

                            left_d = std::max(voxel_sequence_with_cost.first.at(i + 1).voxel_.d_center_ + 0.5 * voxel_sequence_with_cost.first.at(i + 1).voxel_.d_len_, 0.0);
                            right_d = voxel_sequence_with_cost.first.at(i + 1).voxel_.d_center_ - 0.5 * voxel_sequence_with_cost.first.at(i + 1).voxel_.d_len_;
                            voxel_sequence_with_cost.first.at(i + 1).voxel_.d_center_ = 0.5 * (left_d + right_d);
                            voxel_sequence_with_cost.first.at(i + 1).voxel_.d_len_ = left_d - right_d;
                        } else {
                            throw;
                        }
                        break;
                    }
                }
            }
        }
        return kSuccess;
    }

    // 进行图搜索
    ErrorType graphSearch(const std::vector<std::vector<std::shared_ptr<Node>>> &voxel_graph, std::vector<std::pair<std::vector<Node>, double>> &voxel_sequences_with_cost) const {
        // 初始化
        voxel_sequences_with_cost.clear();
        std::vector<std::pair<std::vector<Node>, double>> tmp_voxel_sequences;
        // 首先确定根节点
        std::shared_ptr<Node> root_node_ptr = voxel_graph.front().front();
        // 遍历目标节点
        for (int index = 0; index < voxel_graph.back().size(); index++) {
            // 得到目标节点
            std::shared_ptr<Node> goal_node_ptr = voxel_graph.back().at(index);
            // 初始化
            for (int i = 0; i < voxel_graph.size(); i++) {
                for (int j = 0; j < voxel_graph.at(i).size(); j++) {
                    voxel_graph.at(i).at(j)->is_open_ = false;
                    voxel_graph.at(i).at(j)->is_closed_ = false;
                    voxel_graph.at(i).at(j)->cost_ = 0.0;
                    voxel_graph.at(i).at(j)->pre_node_ptr_ = nullptr;
                }
            }
            // 初始化开集合
            std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, std::less<std::shared_ptr<Node>> > open_set;
            open_set.push(root_node_ptr);
            // 从根节点开始搜索
            while (!open_set.empty()) {
                // 得到搜索列表中最小的index作为当前节点,并从开集合中删除
                std::shared_ptr<Node> current_node_ptr = open_set.top();
                open_set.pop();
                // 将当前节点加入闭集合
                current_node_ptr->close();
                // 判断当前节点是否为终点
                if (goal_node_ptr == current_node_ptr) {
                    // 当前节点为终点,结束搜索
                    break;
                }
                // 如果当前节点不是终点,搜索其的邻居
                for (auto neighbor_node_ptr: current_node_ptr->children_) {
                    // 判断邻居节点是否在闭集合内
                    if (neighbor_node_ptr.first->is_closed_) {
                        // 存在于闭集合内
                        continue;
                    }
                    // 得到邻居的损失
                    double neighbor_cost = current_node_ptr->cost_ + neighbor_node_ptr.second;
                    // 判断邻居节点是否在开集合内
                    if (neighbor_node_ptr.first->is_open_) {
                        // 在开集合内
                        if (neighbor_cost < neighbor_node_ptr.first->cost_) {
                            neighbor_node_ptr.first->pre_node_ptr_ = current_node_ptr;
                            neighbor_node_ptr.first->cost_ = neighbor_cost;
                        }
                    } else {
                        // 不在开集合内
                        // 加入开集合
                        neighbor_node_ptr.first->pre_node_ptr_ = current_node_ptr;
                        neighbor_node_ptr.first->cost_ = neighbor_cost;
                        neighbor_node_ptr.first->open();
                        open_set.push(neighbor_node_ptr.first);
                    }
                }
            }
            // 开始生成最终路径
            // 判断是否生成了最终路径
            if (goal_node_ptr->pre_node_ptr_ == nullptr) {
                continue;
            }
            // 生成了最终路径
            std::vector<Node> voxel_sequence;
            while (goal_node_ptr != nullptr) {
                voxel_sequence.push_back(*goal_node_ptr);
                if(!goal_node_ptr->is_closed_) {
                    throw;
                }
                goal_node_ptr = goal_node_ptr->pre_node_ptr_;
            }
            // 反转路径
            reverse(voxel_sequence.begin(),voxel_sequence.end());
            // 进行保存
            tmp_voxel_sequences.push_back(std::make_pair(voxel_sequence, voxel_sequence.back().cost_));
        }
        // 判断最终生成结果
        if (tmp_voxel_sequences.size() == 0) {
            return kWrongStatus;
        }
        // 进行筛选，每个行为只有一个最小cost的voxel序列
        for (auto behavior: {LateralBehavior::kLaneKeeping, LateralBehavior::kLaneChangeLeft, LateralBehavior::kLaneChangeRight}) {
            double min_cost = std::numeric_limits<double>::max();
            std::vector<Node> selected_voxel_sequence;
            for (auto voxel_sequence: tmp_voxel_sequences) {
                if (voxel_sequence.first.back().voxel_.behavior_ == behavior && voxel_sequence.second < min_cost) {
                    min_cost = voxel_sequence.second;
                    selected_voxel_sequence = voxel_sequence.first;
                }
            }
            if (selected_voxel_sequence.size() > 0) {
                voxel_sequences_with_cost.push_back(std::make_pair(selected_voxel_sequence, min_cost));
            }
        }
        return kSuccess;
    }

    // 进行图构建
    ErrorType buildGraph(const SemanticVehicle &semantic_ego_vehicle, const std::vector<std::vector<Voxel>> &voxels, std::vector<std::vector<std::shared_ptr<Node>>> &voxel_graph) const {
        // 初始化
        voxel_graph.clear();
        // 得到frenet变换
        StateTransformer stf(semantic_ego_vehicle.lane);
        // 得到自身车辆frenet state
        FrenetState init_ego_vehicle_fs;
        if (stf.GetFrenetStateFromState(semantic_ego_vehicle.vehicle.state(), &init_ego_vehicle_fs) != kSuccess) {
            throw;
        }
        // 得到车辆的初始d,初始d速度
        double init_d = init_ego_vehicle_fs.vec_dt[0];
        double init_dd = init_ego_vehicle_fs.vec_dt[1];
        // 得到总步数
        int total_steps = this->time_stamps_.size() - 1;
        for (int step = 0; step < total_steps; step++) {
            // 生成图的每一层
            std::vector<std::shared_ptr<Node>> voxel_graph_layer;
            // 判断是否为第一层
            if (step == 0) {
                for (auto voxel: voxels.at(step)) {
                    if (voxel.behavior_ != LateralBehavior::kLaneKeeping) {
                        continue;
                    }
                    // 构建节点
                    std::shared_ptr<Node> node_ptr = std::make_shared<Node>();
                    node_ptr->voxel_ = voxel;
                    // 进行保存
                    voxel_graph_layer.push_back(node_ptr);
                }
            } else {
                // 得到图的上一层
                std::vector<std::shared_ptr<Node>> last_voxel_graph_layer = voxel_graph.back();
                // 遍历当前层全部栅格
                for (auto voxel: voxels.at(step)) {
                    // 构建节点
                    std::shared_ptr<Node> node_ptr = std::make_shared<Node>();
                    node_ptr->voxel_ = voxel;
                    // 判断其与上一层节点的连接关系
                    for (auto last_layer_node_ptr: last_voxel_graph_layer) {
                        // 首先判断当前节点是否为中道节点
                        if (voxel.behavior_ == LateralBehavior::kLaneKeeping) {
                            // 中道节点的父节点必须是中道节点
                            if (last_layer_node_ptr->voxel_.behavior_ != LateralBehavior::kLaneKeeping) {
                                continue;
                            }
                            // 判断s轴重叠部分是否大于阈值
                            double s_intersection_len = std::min(voxel.s_center_ + 0.5 * voxel.s_len_, last_layer_node_ptr->voxel_.s_center_ + 0.5 * last_layer_node_ptr->voxel_.s_len_) - std::max(voxel.s_center_ - 0.5 * voxel.s_len_, last_layer_node_ptr->voxel_.s_center_ - 0.5 * last_layer_node_ptr->voxel_.s_len_);
                            if (s_intersection_len <= 0) {
                                continue;
                            }
                            // 计算代价
                            double temporal_cost = 1.0 - (2.0 * s_intersection_len / std::pow(this->time_stamps_.at(step), 2.0)) / (2.0 * semantic_ego_vehicle.vehicle.param().max_longitudinal_acc());
                            double cost = temporal_cost;
                            // 更新父节点
                            node_ptr->parents_.push_back(std::make_pair(last_layer_node_ptr, cost));
                            last_layer_node_ptr->children_.push_back(std::make_pair(node_ptr, cost));
                        } else if (voxel.behavior_ == last_layer_node_ptr->voxel_.behavior_) {
                            // 判断s轴重叠部分是否大于阈值
                            double s_intersection_len = std::min(voxel.s_center_ + 0.5 * voxel.s_len_, last_layer_node_ptr->voxel_.s_center_ + 0.5 * last_layer_node_ptr->voxel_.s_len_) - std::max(voxel.s_center_ - 0.5 * voxel.s_len_, last_layer_node_ptr->voxel_.s_center_ - 0.5 * last_layer_node_ptr->voxel_.s_len_);
                            if (s_intersection_len <= 0) {
                                continue;
                            }
                            // 计算代价
                            double temporal_cost = 1.0 - (2.0 * s_intersection_len / std::pow(this->time_stamps_.at(step), 2.0)) / (2.0 * semantic_ego_vehicle.vehicle.param().max_longitudinal_acc());
                            double cost = temporal_cost;
                            if (cost > 0.6) {
                                continue;
                            }
                            // 更新父节点
                            node_ptr->parents_.push_back(std::make_pair(last_layer_node_ptr, cost));
                            last_layer_node_ptr->children_.push_back(std::make_pair(node_ptr, cost));
                        } else {
                            if (last_layer_node_ptr->voxel_.behavior_ != LateralBehavior::kLaneKeeping) {
                                continue;
                            }
                            // 判断d轴重叠部分是否大于阈值
                            if (voxel.behavior_ == LateralBehavior::kLaneChangeLeft) {
                                double max_d = init_d + this->calcMaxDist(init_dd, semantic_ego_vehicle.vehicle.param().max_lateral_vel(), semantic_ego_vehicle.vehicle.param().max_lateral_acc(), this->time_stamps_.at(step));
                                if (max_d < 0.5 * (this->lane_width_ - semantic_ego_vehicle.vehicle.param().width())) {
                                    continue;
                                }
                            } else if (voxel.behavior_ == LateralBehavior::kLaneChangeRight) {
                                double min_d = init_d + this->calcMaxDist(init_dd, -semantic_ego_vehicle.vehicle.param().max_lateral_vel(), -semantic_ego_vehicle.vehicle.param().max_lateral_acc(), this->time_stamps_.at(step));
                                if (min_d > -0.5 * (this->lane_width_ - semantic_ego_vehicle.vehicle.param().width())) {
                                    continue;
                                }
                            } else {
                                throw;
                            }
                            // 找到上一层中相同行为的节点
                            std::vector<Voxel> same_behavior_voxels_in_last_graph_layer;
                            for (auto last_layer_voxel: voxels.at(step - 1)) {
                                if (last_layer_voxel.behavior_ == voxel.behavior_) {
                                    same_behavior_voxels_in_last_graph_layer.push_back(last_layer_voxel);
                                }
                            }
                            if (same_behavior_voxels_in_last_graph_layer.size() == 0) {
                                continue;
                            }
                            // 找到本层中道路保持的其他节点
                            std::vector<Voxel> lane_keeping_voxels_in_current_layer;
                            for (auto current_layer_voxel: voxels.at(step)) {
                                if (current_layer_voxel.behavior_ == LateralBehavior::kLaneKeeping) {
                                    lane_keeping_voxels_in_current_layer.push_back(current_layer_voxel);
                                }
                            }
                            if (lane_keeping_voxels_in_current_layer.size() == 0) {
                                continue;
                            }
                            // 判断s轴重叠部分是否大于阈值
                            double s_intersection_len = 0.0;
                            for (auto last_layer_voxel: same_behavior_voxels_in_last_graph_layer) {
                                for (auto current_layer_voxel: lane_keeping_voxels_in_current_layer) {
                                    double min_boundary = std::max({voxel.s_center_ - 0.5 * voxel.s_len_, current_layer_voxel.s_center_ - 0.5 * current_layer_voxel.s_len_, last_layer_node_ptr->voxel_.s_center_ - 0.5 * last_layer_node_ptr->voxel_.s_len_, last_layer_voxel.s_center_ - 0.5 * last_layer_voxel.s_len_});
                                    double max_boundary = std::min({voxel.s_center_ + 0.5 * voxel.s_len_, current_layer_voxel.s_center_ + 0.5 * current_layer_voxel.s_len_, last_layer_node_ptr->voxel_.s_center_ + 0.5 * last_layer_node_ptr->voxel_.s_len_, last_layer_voxel.s_center_ + 0.5 * last_layer_voxel.s_len_});
                                    if (max_boundary - min_boundary > s_intersection_len) {
                                        s_intersection_len = max_boundary - min_boundary;
                                    }
                                }
                            }
                            if (s_intersection_len <= 0) {
                                continue;
                            }
                            // 计算代价
                            double temporal_cost = 1.0 - (2.0 * s_intersection_len / std::pow(this->time_stamps_.at(step), 2.0)) / (2.0 * semantic_ego_vehicle.vehicle.param().max_longitudinal_acc());
                            double cost = temporal_cost;
                            if (cost > 0.6) {
                                continue;
                            }
                            // 更新父节点
                            node_ptr->parents_.push_back(std::make_pair(last_layer_node_ptr, cost));
                            last_layer_node_ptr->children_.push_back(std::make_pair(node_ptr, cost));
                        }
                    }
                    // 判断新构建节点父节点数量
                    if (node_ptr->parents_.size() > 0) {
                        // 进行保存
                        voxel_graph_layer.push_back(node_ptr);
                    }
                }
            }
            // 判断是否生成成功
            if (voxel_graph_layer.size() == 0) {
                // 生成失败
                voxel_graph.clear();
                return kWrongStatus;
            } else {
                // 生成成功
                voxel_graph.push_back(voxel_graph_layer);
                // 输出调试信息
                std::cout << "======== step " << step << " has " << voxel_graph_layer.size() << " nodes ========" << std::endl;
                for (auto voxel_node_ptr: voxel_graph_layer) {
                    std::cout << "node " << voxel_node_ptr->voxel_.t_center_ << " " << static_cast<int>(voxel_node_ptr->voxel_.behavior_);
                    for (auto parent: voxel_node_ptr->parents_) {
                        std::cout << ", parent: " << parent.first->voxel_.t_center_ << " " << static_cast<int>(parent.first->voxel_.behavior_) << " " << parent.second;
                    }
                    std::cout << std::endl;
                }
            }
        }
        // 可视化树
        return kSuccess;
    };

    // 生成栅格
    ErrorType voxelGeneration(const SemanticVehicle &semantic_ego_vehicle, const SemanticLaneSet &semantic_lane_set, const SemanticVehicleSet &surr_semantic_vehicle_set, std::vector<std::vector<Voxel>> &voxels) const {
        // 初始化
        voxels.clear();
        // 首先得到自身车道的id
        int ego_lane_id = semantic_ego_vehicle.nearest_lane_id;
        // 得到左右车道的id
        int left_lane_id = semantic_lane_set.semantic_lanes.at(ego_lane_id).l_lane_id;
        int right_lane_id = semantic_lane_set.semantic_lanes.at(ego_lane_id).r_lane_id;
        // 找到正后方车辆，不参与计算
        int rear_semantic_vehicle_id;
        double rear_dist = std::numeric_limits<double>::max();
        for (auto semantic_vehicle: surr_semantic_vehicle_set.semantic_vehicles) {
            if (semantic_vehicle.second.nearest_lane_id == ego_lane_id) {
                // 判断在自身车辆后方
                if (semantic_vehicle.second.arc_len_onlane < semantic_ego_vehicle.arc_len_onlane) {
                    // 后方
                    if (std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane) < rear_dist) {
                        rear_dist = std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane);
                        rear_semantic_vehicle_id = semantic_vehicle.first;
                    }
                }
            }
        }
        std::cout << "rear_semantic_vehicle_id: " << rear_semantic_vehicle_id << std::endl;
        SemanticVehicleSet semantic_vehicle_set;
        for (auto semantic_vehicle: surr_semantic_vehicle_set.semantic_vehicles) {
            if (semantic_vehicle.first != rear_semantic_vehicle_id) {
                semantic_vehicle_set.semantic_vehicles.insert(semantic_vehicle);
            }
        }
        // 得到frenet变换
        StateTransformer stf(semantic_ego_vehicle.lane);
        // 得到自身车辆frenet state
        FrenetState init_ego_vehicle_fs;
        if (stf.GetFrenetStateFromState(semantic_ego_vehicle.vehicle.state(), &init_ego_vehicle_fs) != kSuccess) {
            throw;
        }
        // 得到车辆的初始s，初始s速度
        double init_s = init_ego_vehicle_fs.vec_s[0];
        double init_ds = init_ego_vehicle_fs.vec_s[1];
        // 得到车辆的初始d,初始d速度
        double init_d = init_ego_vehicle_fs.vec_dt[0];
        double init_dd = init_ego_vehicle_fs.vec_dt[1];
        std::cout << "init s: " << init_s << ", init d: " << init_d << ", angle: " << semantic_ego_vehicle.angle_diff_onlane << ", vel: " << semantic_ego_vehicle.vehicle.state().velocity << ", init ds: " << init_ds << ", init dd: " << init_dd << std::endl;
        // 得到总步数
        int total_steps = this->time_stamps_.size() - 1;
        for (int step = 0; step < total_steps; step++) {
            std::vector<Voxel> step_voxels;
            // 得到时间段
            double start_time = this->time_stamps_.at(step);
            double end_time = this->time_stamps_.at(step + 1);
            std::cout << "++++++++++++++++ time from " << start_time << " to " << end_time << " ++++++++++++++++" << std::endl;
            // 计算时间段的s变化范围
            double min_s = init_s + this->calcMinDist(init_ds, 0.0, -semantic_ego_vehicle.vehicle.param().max_longitudinal_acc(), start_time);
            double max_s = init_s + this->calcMaxDist(init_ds, this->max_longitude_vel_, semantic_ego_vehicle.vehicle.param().max_longitudinal_acc(), end_time);
            // 计算时间段内d变化范围
            double left_d_start = init_d + this->calcMaxDist(init_dd, semantic_ego_vehicle.vehicle.param().max_lateral_vel(), semantic_ego_vehicle.vehicle.param().max_lateral_acc(), start_time);
            double left_d_end = init_d + this->calcMaxDist(init_dd, semantic_ego_vehicle.vehicle.param().max_lateral_vel(), semantic_ego_vehicle.vehicle.param().max_lateral_acc(), end_time);
            double right_d_start = init_d + this->calcMaxDist(init_dd, -semantic_ego_vehicle.vehicle.param().max_lateral_vel(), -semantic_ego_vehicle.vehicle.param().max_lateral_acc(), start_time);
            double right_d_end = init_d + this->calcMaxDist(init_dd, -semantic_ego_vehicle.vehicle.param().max_lateral_vel(), -semantic_ego_vehicle.vehicle.param().max_lateral_acc(), end_time);
            double expected_left_d = 0.5 * (this->lane_width_ - semantic_ego_vehicle.vehicle.param().width());
            double expected_right_d = -0.5 * (this->lane_width_ - semantic_ego_vehicle.vehicle.param().width());
            double left_d, right_d;
            if (step < this->min_valid_num_ || std::min(expected_left_d, left_d_start) - std::max(expected_right_d, right_d_start) < 0 || std::min(expected_left_d, left_d_end) - std::max(expected_right_d, right_d_end) < 0) {
                left_d = std::max(left_d_start, left_d_end);
                right_d = std::min(right_d_start, right_d_end);
            } else {
                left_d = expected_left_d;
                right_d = expected_right_d;
            }
            std::cout << "step " << step << ", s range: " << min_s << ", " << max_s << ", d range: " << right_d << ", " << left_d << std::endl;
            // 得到自身道路s可行区间
            if (ego_lane_id != kInvalidLaneId) {
                std::cout << "-------- ego lane --------" << std::endl;
                std::vector<int> corresponding_lane_ids = {ego_lane_id};
                // if (left_d > expected_left_d && left_lane_id != kInvalidLaneId) {
                //     corresponding_lane_ids.push_back(left_lane_id);
                // }
                // if (right_d < expected_right_d && right_lane_id != kInvalidLaneId) {
                //     corresponding_lane_ids.push_back(right_lane_id);
                // }
                std::vector<std::pair<double, double>> ego_lane_valid_s_intervals = this->getValidIntervals(min_s, max_s, start_time, end_time, semantic_ego_vehicle, semantic_vehicle_set, corresponding_lane_ids);
                // 进行栅格构建
                for (auto s_interval: ego_lane_valid_s_intervals) {
                    Voxel voxel;
                    voxel.behavior_ = LateralBehavior::kLaneKeeping;
                    voxel.s_center_ = 0.5 * (s_interval.first + s_interval.second);
                    voxel.s_len_ = s_interval.second - s_interval.first;
                    voxel.d_center_ = 0.5 * (left_d + right_d);
                    voxel.d_len_ = left_d - right_d;
                    if (voxel.s_len_ <= 0.0 || voxel.d_len_ <= 0.0) {
                        throw;
                    }
                    voxel.t_center_ = 0.5 * (start_time + end_time);
                    voxel.t_len_ = end_time - start_time;
                    step_voxels.push_back(voxel);
                }
            }
            // 得到左侧道路s可行区间
            if (left_lane_id != kInvalidLaneId) {
                std::cout << "-------- left lane --------" << std::endl;
                std::vector<int> corresponding_lane_ids = {left_lane_id};
                std::vector<std::pair<double, double>> left_lane_valid_s_intervals = this->getValidIntervals(min_s, max_s, start_time, end_time, semantic_ego_vehicle, semantic_vehicle_set, corresponding_lane_ids);
                // 进行栅格构建
                for (auto s_interval: left_lane_valid_s_intervals) {
                    Voxel voxel;
                    voxel.behavior_ = LateralBehavior::kLaneChangeLeft;
                    voxel.s_center_ = 0.5 * (s_interval.first + s_interval.second);
                    voxel.s_len_ = s_interval.second - s_interval.first;
                    voxel.d_center_ = this->lane_width_;
                    voxel.d_len_ = this->lane_width_ - semantic_ego_vehicle.vehicle.param().width();
                    if (voxel.s_len_ <= 0.0) {
                        throw;
                    }
                    voxel.t_center_ = 0.5 * (start_time + end_time);
                    voxel.t_len_ = end_time - start_time;
                    step_voxels.push_back(voxel);
                }
            }
            // 得到右侧道路s可行区间
            if (right_lane_id != kInvalidLaneId) {
                std::cout << "-------- right lane --------" << std::endl;
                std::vector<int> corresponding_lane_ids = {right_lane_id};
                std::vector<std::pair<double, double>> right_lane_valid_s_intervals = this->getValidIntervals(min_s, max_s, start_time, end_time, semantic_ego_vehicle, semantic_vehicle_set, corresponding_lane_ids);
                // 进行栅格构建
                for (auto s_interval: right_lane_valid_s_intervals) {
                    Voxel voxel;
                    voxel.behavior_ = LateralBehavior::kLaneChangeRight;
                    voxel.s_center_ = 0.5 * (s_interval.first + s_interval.second);
                    voxel.s_len_ = s_interval.second - s_interval.first;
                    voxel.d_center_ = -this->lane_width_;
                    voxel.d_len_ = this->lane_width_ - semantic_ego_vehicle.vehicle.param().width();
                    if (voxel.s_len_ <= 0.0) {
                        throw;
                    }
                    voxel.t_center_ = 0.5 * (start_time + end_time);
                    voxel.t_len_ = end_time - start_time;
                    step_voxels.push_back(voxel);
                }
            }
            if (step_voxels.size() == 0) {
                voxels.clear();
                return kWrongStatus;
            } else {
                voxels.push_back(step_voxels);
            }
        }
        // 进行可视化调试
        return kSuccess;
    }

    // 计算最小距离
    double calcMinDist(double v0, double min_v, double max_dec, double time) const {
        double time_to_min = min_v - v0 / max_dec;
        if (time < time_to_min) {
            return v0 * time + 0.5 * max_dec * time * time;
        } else {
            return v0 * v0 / (2.0 * -max_dec);
        }

    }

    // 计算最大距离
    double calcMaxDist(double v0, double max_v, double max_acc, double time) const {
        double time_to_max = (max_v - v0) / max_acc;
        if (time < time_to_max) {
            return v0 * time + 0.5 * max_acc * time * time;
        } else {
            return v0 * time_to_max + 0.5 * max_acc * time_to_max * time_to_max + (time - time_to_max) * max_v;
        }
    }

    // 得到有效区间
    std::vector<std::pair<double, double>> getValidIntervals(double min_s, double max_s, double start_time, double end_time, const SemanticVehicle &semantic_ego_vehicle, const SemanticVehicleSet &semantic_vehicle_set, const std::vector<int> &corresponding_lane_ids) const {
        std::vector<std::pair<double, double>> valid_s_segments;
        boost::icl::interval_set<double> raw_s_intervals, vehicle_s_intervals;
        raw_s_intervals += boost::icl::continuous_interval<double>::closed(min_s, max_s);
        for (auto corresponding_lane_id: corresponding_lane_ids) {
            for (auto semantic_vehicle: semantic_vehicle_set.semantic_vehicles) {
                if (semantic_vehicle.second.nearest_lane_id != corresponding_lane_id) {
                    continue;
                }
                // 计算障碍物车辆的移动距离
                double vehicle_min_s = semantic_vehicle.second.arc_len_onlane + semantic_vehicle.second.vehicle.state().velocity * start_time;
                double vehicle_max_s = semantic_vehicle.second.arc_len_onlane + semantic_vehicle.second.vehicle.state().velocity * end_time;
                // 增加障碍物车辆长度和自身车辆的长度
                vehicle_min_s -= semantic_vehicle.second.vehicle.param().rear_suspension() + semantic_ego_vehicle.vehicle.param().front_suspension() + semantic_ego_vehicle.vehicle.param().wheel_base();
                vehicle_max_s += semantic_ego_vehicle.vehicle.param().rear_suspension() + semantic_vehicle.second.vehicle.param().front_suspension() + semantic_vehicle.second.vehicle.param().wheel_base();
                vehicle_s_intervals += boost::icl::continuous_interval<double>::open(vehicle_min_s, vehicle_max_s);
                std::cout << "vehicle: " << semantic_vehicle.second.vehicle.id() << ": arc len " << semantic_vehicle.second.arc_len_onlane << ", s range " << vehicle_min_s << ", to " << vehicle_max_s << std::endl;
            }
        }
        boost::icl::interval_set<double> valid_s_intervals = raw_s_intervals - vehicle_s_intervals;
        std::cout << "raw s intervals: " << raw_s_intervals << std::endl;
        std::cout << "vehicle s intervals: " << vehicle_s_intervals << std::endl;
        std::cout << "valid s intervals: " << valid_s_intervals << std::endl;
        for (auto valid_s_interval: valid_s_intervals) {
            valid_s_segments.push_back(std::make_pair(valid_s_interval.lower() - kBigEPS, valid_s_interval.upper() + kBigEPS));
        }
        return valid_s_segments;
    }
};

#endif  // TRAJECTORY_PLANNING_HPP_
