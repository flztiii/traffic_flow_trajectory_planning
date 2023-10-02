#ifndef LOCAL_PLANNING_HPP_
#define LOCAL_PLANNING_HPP_

#include "calculation.hpp"
#include "vehicle.hpp"
#include "simulator/Lane.h"
#include "simulator/Vehicle.h"
#include "semantic.hpp"
#include "trajectory_planner.hpp"

// 局部规划器
class LocalPlanner {
 public:
    // 构造函数
    LocalPlanner() {
        // 初始化参数
        this->max_time_horizon_ = 5.0;
        this->range_ = 100.0;
        // 初始化行为规划器
        this->trajectory_planner_ptr_ = std::make_shared<TrajectoryPlanner>(this->max_time_horizon_);
        ros::NodeHandle nh("~");
        this->debug_semantic_lane_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/local_planning/debug_semantic_lane_vis", 10, true);
        this->debug_semantic_vehicle_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/local_planning/debug_semantic_vehicle_vis", 10, true);
        this->debug_trajectory_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/local_planning/debug_trajectory_vis", 10, true);
    };

    // 析构函数
    ~LocalPlanner() {};

    // 进行规划
    ErrorType planOnce(const Vehicle &raw_ego_vehicle, const std::vector<simulator::Lane> &raw_lanes, const std::vector<simulator::Vehicle> other_vehicles, const int aim_lane_id, double planning_horizon, VehicleControlSignal &control_signal) {
        // 第一步，将输入进行进行处理
        // 得到语义道路信息
        SemanticLaneSet semantic_lane_set = this->updateSemanticLaneSet(raw_ego_vehicle.state(), raw_lanes);
        // 得到自身车辆语义信息
        Vehicle local_ego_vehicle = raw_ego_vehicle;
        State local_ego_vehicle_state = local_ego_vehicle.state();
        local_ego_vehicle.set_state(local_ego_vehicle_state);
        SemanticVehicle semantic_ego_vehicle;
        semantic_ego_vehicle.vehicle = local_ego_vehicle;
        if (this->GetNearestLaneIdUsingState(semantic_ego_vehicle.vehicle.state().ToXYTheta(), semantic_lane_set, &semantic_ego_vehicle.nearest_lane_id, &semantic_ego_vehicle.dist_to_lane, &semantic_ego_vehicle.arc_len_onlane, &semantic_ego_vehicle.angle_diff_onlane) != kSuccess) {
            throw;
        } else {
            semantic_ego_vehicle.lane = semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).lane;
        }
        // 得到周围车辆语义信息
        SemanticVehicleSet surr_semantic_vehicle_set = this->updateSurrSemanticVehicleSet(semantic_ego_vehicle, semantic_lane_set, other_vehicles);
        // 判断当前场景是否发生碰撞
        if (this->collisionJudge(semantic_ego_vehicle, semantic_lane_set, surr_semantic_vehicle_set)) {
            return kWrongStatus;
        }
        // 得到期望行为
        LateralBehavior desire_behavior;
        if (aim_lane_id == semantic_ego_vehicle.nearest_lane_id) {
            desire_behavior = LateralBehavior::kLaneKeeping;
        } else if (aim_lane_id == semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).l_lane_id && semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).l_change_avbl) {
            desire_behavior = LateralBehavior::kLaneChangeLeft;
        } else if (aim_lane_id == semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).r_lane_id && semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).r_change_avbl) {
            desire_behavior = LateralBehavior::kLaneChangeRight;
        } else {
            desire_behavior = LateralBehavior::kUndefined;
        }
        // 可视化全局路径曲率
        {
            std::ofstream global_path_file(ros::package::getPath("local_planning") + "/data/global_path.csv", std::ios::out);
            std::vector<double> samples = linspace(semantic_ego_vehicle.lane.begin(), semantic_ego_vehicle.lane.end(), 1000);
            for (double sample: samples) {
                double cur, theta;
                semantic_ego_vehicle.lane.GetCurvatureByArcLength(sample, &cur);
                semantic_ego_vehicle.lane.GetOrientationByArcLength(sample, &theta);
                global_path_file << sample << "," << theta << "," << cur << std::endl;
            }
            global_path_file.close();
        }
        // 第二步，进行轨迹的生成
        FrenetBezierTrajectory<5, 2> planned_trajectory;
        if (this->trajectory_planner_ptr_->planOnce<5, 2>(semantic_ego_vehicle, semantic_lane_set, surr_semantic_vehicle_set, desire_behavior, planned_trajectory) != kSuccess) {
            // 规划失败
            return kWrongStatus;
        }
        // 进行可视化
        {
            // 删除之前的可视化
            visualization_msgs::MarkerArray delete_marker_array;
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "odom";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.action = visualization_msgs::Marker::DELETEALL;
            delete_marker.id = 0;
            delete_marker_array.markers.push_back(delete_marker);
            this->debug_trajectory_vis_.publish(delete_marker_array);
            // 进行可视化
            visualization_msgs::MarkerArray marker_array;
            int id = 0;
            std::vector<double> samples;
            GetRangeVector<double>(planned_trajectory.begin(), planned_trajectory.end(), 0.1, true, &samples);
            for (auto sample: samples) {
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
                marker.scale.x = 0.05;
                State state;
                if (planned_trajectory.GetState(sample, &state) != kSuccess) {
                    throw;
                }
                Vehicle vehicle(semantic_ego_vehicle.vehicle.param(), state);
                vec_E<Vec2f> vertices;
                vehicle.RetVehicleVertices(&vertices);
                for (int i = 0; i <= vertices.size(); i++) {
                    geometry_msgs::Point point;
                    point.x = vertices.at(i % vertices.size())(0);
                    point.y = vertices.at(i % vertices.size())(1);
                    marker.points.push_back(point);
                }
                marker_array.markers.push_back(marker);
            }
            this->debug_trajectory_vis_.publish(marker_array);
        }
        // 进行数据存储
        {
            // 数据生成
            std::vector<double> t = linspace(planned_trajectory.begin(), planned_trajectory.end(), static_cast<int>((planned_trajectory.end() - planned_trajectory.begin()) / 0.01));
            std::vector<double> pos_long, pos_lat, vels_long, vels_lat, accs_long, accs_lat, jerks_long, jerks_lat;
            for (int i = 0; i < t.size(); i++) {
                Vec2f pos, vel, acc, jerk;
                planned_trajectory.bezier_spline().evaluate(t[i], 0, &pos);
                planned_trajectory.bezier_spline().evaluate(t[i], 1, &vel);
                planned_trajectory.bezier_spline().evaluate(t[i], 2, &acc);
                planned_trajectory.bezier_spline().evaluate(t[i], 3, &jerk);
                pos_long.push_back(pos[0]);
                pos_lat.push_back(pos[1]);
                vels_long.push_back(vel[0]);
                vels_lat.push_back(vel[1]);
                accs_long.push_back(acc[0]);
                accs_lat.push_back(acc[1]);
                jerks_long.push_back(jerk[0]);
                jerks_lat.push_back(jerk[1]);
            }
            // 数据保存
            std::string record_data_file_path = ros::package::getPath("local_planning") + "/data/test_record.csv";
            std::ofstream record_data_file(record_data_file_path, std::ios::out);
            for (int i = 0; i < t.size(); i++) {
                record_data_file << t[i] << "," << pos_long[i] << "," << pos_lat[i] << "," << vels_long[i] << "," << vels_lat[i] << "," << accs_long[i] << "," << accs_lat[i] << "," << jerks_long[i] << "," << jerks_lat[i] << "\n";
            }
            record_data_file.close();
        }
        // 最后一步，得到规划结果
        State desire_state;
        if (planned_trajectory.GetState(planning_horizon, &desire_state) != kSuccess) {
            throw;
        }
        control_signal = VehicleControlSignal(desire_state);
        return kSuccess;
    };

 private:
    std::shared_ptr<TrajectoryPlanner> trajectory_planner_ptr_;
    ros::Publisher debug_semantic_lane_vis_;
    ros::Publisher debug_semantic_vehicle_vis_;
    ros::Publisher debug_trajectory_vis_;
    double max_time_horizon_;
    double range_;

    // 比较器
    struct Comparer {
        bool operator()(const std::tuple<double, double, double, int>& a, const std::tuple<double, double, double, int>& b) const {
            return fabs(std::get<0>(a)) < fabs(std::get<0>(b));
        }
    };

    // 碰撞判断
    bool collisionJudge(const SemanticVehicle &semantic_ego_vehicle, const SemanticLaneSet &semantic_lane_set, const SemanticVehicleSet &surr_semantic_vehicle_set) const {
        bool is_collision = false;
        // 与周围车辆判断碰撞
        for (auto semantic_vehicle: surr_semantic_vehicle_set.semantic_vehicles) {
            if (this->checkCollisionUsingState(semantic_ego_vehicle.vehicle.param(), semantic_ego_vehicle.vehicle.state(), semantic_vehicle.second.vehicle.param(), semantic_vehicle.second.vehicle.state(), &is_collision) != kSuccess) {
                throw;
            }
            if (is_collision) {
                break;
            }
        }
        // 与车道边界判断碰撞
        if (!is_collision) {
            int left_lane_id = semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).l_lane_id;
            int right_lane_id = semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).r_lane_id;
            // 得到frenet变换
            StateTransformer stf(semantic_ego_vehicle.lane);
            vec_E<Vec2f> vertices;
            if (semantic_ego_vehicle.vehicle.RetVehicleVertices(&vertices) != kSuccess) {
                throw;
            }
            // 得到车辆四个角点
            if (left_lane_id == kInvalidLaneId) {
                // 左侧道路不存在
                for (auto vertex: vertices) {
                    Vec2f fx_vertex;
                    if (stf.GetFrenetPointFromPoint(vertex, &fx_vertex) != kSuccess) {
                        throw;
                    }
                    if (fx_vertex[1] > 2.0) {
                        is_collision = true;
                        break;
                    }
                }
            } else if (right_lane_id == kInvalidLaneId) {
                // 右侧道路不存在
                for (auto vertex: vertices) {
                    Vec2f fx_vertex;
                    if (stf.GetFrenetPointFromPoint(vertex, &fx_vertex) != kSuccess) {
                        throw;
                    }
                    if (fx_vertex[1] < -2.0) {
                        is_collision = true;
                        break;
                    }
                }
            }
        }
        return is_collision;
    }

    ErrorType checkCollisionUsingState(const VehicleParam &param_a, const State &state_a, const VehicleParam &param_b, const State &state_b, bool *res) const {
        OrientedBoundingBox2D obb_a, obb_b;
        this->getOrientedBoundingBoxForVehicleUsingState(param_a, state_a, &obb_a);
        this->getOrientedBoundingBoxForVehicleUsingState(param_b, state_b, &obb_b);
        *res = CheckIfOrientedBoundingBoxIntersect(obb_a, obb_b);
        return kSuccess;
    }

    ErrorType getOrientedBoundingBoxForVehicleUsingState(const VehicleParam &param, const State &s, OrientedBoundingBox2D *obb) const {
        double cos_theta = cos(s.angle);
        double sin_theta = sin(s.angle);
        obb->x = s.vec_position[0] + param.d_cr() * cos_theta;
        obb->y = s.vec_position[1] + param.d_cr() * sin_theta;
        obb->angle = s.angle;
        obb->length = param.length();
        obb->width = param.width();
        return kSuccess;
    }


    // 周围车辆信息转换
    SemanticVehicleSet updateSurrSemanticVehicleSet(const SemanticVehicle &semantic_ego_vehicle, const SemanticLaneSet &semantic_lane_set, const std::vector<simulator::Vehicle> other_vehicles) const {
        SemanticVehicleSet tmp_semantic_vehicle_set;
        for (auto vehicle: other_vehicles) {
            SemanticVehicle semantic_vehicle;
            // 首先设置车辆信息
            semantic_vehicle.vehicle.set_id(vehicle.id);
            // 车辆参数
            VehicleParam vehicle_param;
            vehicle_param.set_width(vehicle.width);
            vehicle_param.set_length(vehicle.length);
            vehicle_param.set_wheel_base(vehicle_param.length() - vehicle_param.front_suspension() - vehicle_param.rear_suspension());
            vehicle_param.set_d_cr(0.5 * vehicle_param.length() - vehicle_param.rear_suspension());
            semantic_vehicle.vehicle.set_param(vehicle_param);
            // 车辆状态
            State vehicle_state;
            geometry_msgs::Point point;
            vehicle_state.vec_position(0) = vehicle.center.x - vehicle_param.d_cr() * cos(vehicle.orientation);
            vehicle_state.vec_position(1) = vehicle.center.y - vehicle_param.d_cr() * sin(vehicle.orientation);
            vehicle_state.angle = vehicle.orientation;
            vehicle_state.velocity = vehicle.speed;
            semantic_vehicle.vehicle.set_state(vehicle_state);
            // 根据坐标判断车辆是否属于考虑范围
            double dist = sqrt(std::pow(vehicle_state.vec_position(0) - semantic_ego_vehicle.vehicle.state().vec_position(0), 2.0) + std::pow(vehicle_state.vec_position(1) - semantic_ego_vehicle.vehicle.state().vec_position(1), 2.0));
            if (dist > this->range_) {
                continue;
            }
            // 根据状态判断所在的lane id
            if (this->GetNearestLaneIdUsingState(semantic_vehicle.vehicle.state().ToXYTheta(), semantic_lane_set, &semantic_vehicle.nearest_lane_id, &semantic_vehicle.dist_to_lane, &semantic_vehicle.arc_len_onlane, &semantic_vehicle.angle_diff_onlane) != kSuccess) {
                throw;
            }
            // 根据车道判断是否属于考虑范围
            if (semantic_vehicle.nearest_lane_id == semantic_ego_vehicle.nearest_lane_id || semantic_vehicle.nearest_lane_id == semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).l_lane_id || semantic_vehicle.nearest_lane_id == semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).r_lane_id) {
                // 确定车辆的道路
                semantic_vehicle.lane = semantic_lane_set.semantic_lanes.at(semantic_vehicle.nearest_lane_id).lane;
                // 进行保存
                tmp_semantic_vehicle_set.semantic_vehicles.insert(std::make_pair(semantic_vehicle.vehicle.id(), semantic_vehicle));
            }
        }
        // 再次进行筛选
        SemanticVehicleSet semantic_vehicle_set;
        {
            // 筛选出本车道的前后车
            SemanticVehicle ahead_semantic_vehicle, rear_semantic_vehicle;
            double ahead_dist = std::numeric_limits<double>::max();
            double rear_dist = std::numeric_limits<double>::max();
            for (auto semantic_vehicle: tmp_semantic_vehicle_set.semantic_vehicles) {
                if (semantic_vehicle.second.nearest_lane_id == semantic_ego_vehicle.nearest_lane_id) {
                    // 判断在自身车辆前方还是后方
                    if (semantic_vehicle.second.arc_len_onlane > semantic_ego_vehicle.arc_len_onlane) {
                        // 前方
                        if (std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane) < ahead_dist) {
                            ahead_dist = std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane);
                            ahead_semantic_vehicle = semantic_vehicle.second;
                        }
                    } else {
                        // 后方
                        if (std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane) < rear_dist) {
                            rear_dist = std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane);
                            rear_semantic_vehicle = semantic_vehicle.second;
                        }
                    }
                }
            }
            if (ahead_semantic_vehicle.vehicle.id() != kInvalidAgentId) {
                semantic_vehicle_set.semantic_vehicles.insert(std::make_pair(ahead_semantic_vehicle.vehicle.id(), ahead_semantic_vehicle));
            }
            if (rear_semantic_vehicle.vehicle.id() != kInvalidAgentId) {
                // 认为正后方的车速度不可能超过自身
                State changed_state = rear_semantic_vehicle.vehicle.state();
                changed_state.velocity = std::min(changed_state.velocity, semantic_ego_vehicle.vehicle.state().velocity);
                rear_semantic_vehicle.vehicle.set_state(changed_state);
                semantic_vehicle_set.semantic_vehicles.insert(std::make_pair(rear_semantic_vehicle.vehicle.id(), rear_semantic_vehicle));
            }
        }
        // 筛选左右车道重要车
        int left_lane_id = semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).l_lane_id;
        if (left_lane_id != kInvalidLaneId) {
            // 左侧道存在
            SemanticVehicleSet left_lane_vehicle_set;
            // 筛选出本车道的前后车
            double ahead_arc_len = std::numeric_limits<double>::max();
            double rear_arc_len = -std::numeric_limits<double>::max();
            double ahead_dist = std::numeric_limits<double>::max();
            double rear_dist = std::numeric_limits<double>::max();
            double max_forward = semantic_ego_vehicle.vehicle.state().velocity * this->max_time_horizon_ + 0.5 * semantic_ego_vehicle.vehicle.param().max_longitudinal_acc() * this->max_time_horizon_ * this->max_time_horizon_;
            double min_forward = std::max(0.0, semantic_ego_vehicle.vehicle.state().velocity * this->max_time_horizon_ - 0.5 * semantic_ego_vehicle.vehicle.param().max_longitudinal_acc() * this->max_time_horizon_ * this->max_time_horizon_);
            for (auto semantic_vehicle: tmp_semantic_vehicle_set.semantic_vehicles) {
                if (semantic_vehicle.second.nearest_lane_id == left_lane_id) {
                    // 转化到本车道坐标系下
                    std::tuple<double, double, double> dist_to_lane;
                    if (GetDistanceToLanesUsing3DofState(semantic_vehicle.second.vehicle.Ret3DofState(), semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).lane, dist_to_lane) != kSuccess) {
                        throw;
                    }
                    semantic_vehicle.second.arc_len_onlane = std::get<1>(dist_to_lane);
                    semantic_vehicle.second.dist_to_lane = std::get<0>(dist_to_lane);
                    semantic_vehicle.second.angle_diff_onlane = std::get<2>(dist_to_lane);
                    left_lane_vehicle_set.semantic_vehicles.insert(semantic_vehicle);
                    // 判断是前车还是后车
                    if (semantic_vehicle.second.arc_len_onlane > semantic_ego_vehicle.arc_len_onlane && semantic_vehicle.second.arc_len_onlane + semantic_vehicle.second.vehicle.state().velocity * this->max_time_horizon_ > semantic_ego_vehicle.arc_len_onlane + max_forward) {
                        // 前车
                        if (std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane) < ahead_dist) {
                            ahead_dist = std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane);
                            ahead_arc_len = semantic_vehicle.second.arc_len_onlane;
                        }
                    } else if (semantic_vehicle.second.arc_len_onlane < semantic_ego_vehicle.arc_len_onlane && semantic_vehicle.second.arc_len_onlane + semantic_vehicle.second.vehicle.state().velocity * this->max_time_horizon_ < semantic_ego_vehicle.arc_len_onlane + min_forward) {
                        // 后车
                        if (std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane) < rear_dist) {
                            rear_dist = std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane);
                            rear_arc_len = semantic_vehicle.second.arc_len_onlane;
                        }
                    }
                }
            }
            for (auto semantic_vehicle: left_lane_vehicle_set.semantic_vehicles) {
                if (semantic_vehicle.second.arc_len_onlane < rear_arc_len || semantic_vehicle.second.arc_len_onlane > ahead_arc_len) {
                    continue;
                }
                semantic_vehicle_set.semantic_vehicles.insert(semantic_vehicle);
            }
        }
        int right_lane_id = semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).r_lane_id;
        if (right_lane_id != kInvalidLaneId) {
            // 右侧道存在
            SemanticVehicleSet right_lane_vehicle_set;
            // 筛选出本车道的前后车
            double ahead_arc_len = std::numeric_limits<double>::max();
            double rear_arc_len = -std::numeric_limits<double>::max();
            double ahead_dist = std::numeric_limits<double>::max();
            double rear_dist = std::numeric_limits<double>::max();
            double max_forward = semantic_ego_vehicle.vehicle.state().velocity * this->max_time_horizon_ + 0.5 * semantic_ego_vehicle.vehicle.param().max_longitudinal_acc() * this->max_time_horizon_ * this->max_time_horizon_;
            double min_forward = std::max(0.0, semantic_ego_vehicle.vehicle.state().velocity * this->max_time_horizon_ - 0.5 * semantic_ego_vehicle.vehicle.param().max_longitudinal_acc() * this->max_time_horizon_ * this->max_time_horizon_);
            for (auto semantic_vehicle: tmp_semantic_vehicle_set.semantic_vehicles) {
                if (semantic_vehicle.second.nearest_lane_id == right_lane_id) {
                    // 转换到本车道坐标系下
                    std::tuple<double, double, double> dist_to_lane;
                    if (GetDistanceToLanesUsing3DofState(semantic_vehicle.second.vehicle.Ret3DofState(), semantic_lane_set.semantic_lanes.at(semantic_ego_vehicle.nearest_lane_id).lane, dist_to_lane) != kSuccess) {
                        throw;
                    }
                    semantic_vehicle.second.arc_len_onlane = std::get<1>(dist_to_lane);
                    semantic_vehicle.second.dist_to_lane = std::get<0>(dist_to_lane);
                    semantic_vehicle.second.angle_diff_onlane = std::get<2>(dist_to_lane);
                    right_lane_vehicle_set.semantic_vehicles.insert(semantic_vehicle);
                    // 判断是前车还是后车
                    if (semantic_vehicle.second.arc_len_onlane > semantic_ego_vehicle.arc_len_onlane && semantic_vehicle.second.arc_len_onlane + semantic_vehicle.second.vehicle.state().velocity * this->max_time_horizon_ > semantic_ego_vehicle.arc_len_onlane + max_forward) {
                        // 前车
                        if (std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane) < ahead_dist) {
                            ahead_dist = std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane);
                            ahead_arc_len = semantic_vehicle.second.arc_len_onlane;
                        }
                    } else if (semantic_vehicle.second.arc_len_onlane < semantic_ego_vehicle.arc_len_onlane && semantic_vehicle.second.arc_len_onlane + semantic_vehicle.second.vehicle.state().velocity * this->max_time_horizon_ < semantic_ego_vehicle.arc_len_onlane + min_forward) {
                        // 后车
                        if (std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane) < rear_dist) {
                            rear_dist = std::abs(semantic_vehicle.second.arc_len_onlane - semantic_ego_vehicle.arc_len_onlane);
                            rear_arc_len = semantic_vehicle.second.arc_len_onlane;
                        }
                    }
                }
            }
            for (auto semantic_vehicle: right_lane_vehicle_set.semantic_vehicles) {
                if (semantic_vehicle.second.arc_len_onlane < rear_arc_len || semantic_vehicle.second.arc_len_onlane > ahead_arc_len) {
                    continue;
                }
                semantic_vehicle_set.semantic_vehicles.insert(semantic_vehicle);
            }
        }
        // 进行可视化debug
        {
            // 删除之前的可视化
            visualization_msgs::MarkerArray delete_marker_array;
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "odom";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.action = visualization_msgs::Marker::DELETEALL;
            delete_marker.id = 0;
            delete_marker_array.markers.push_back(delete_marker);
            this->debug_semantic_vehicle_vis_.publish(delete_marker_array);
            // 进行可视化
            visualization_msgs::MarkerArray marker_array;
            int id = 0;
            for(const auto &semantic_vehicle: semantic_vehicle_set.semantic_vehicles) {
                // 显示边界框
                visualization_msgs::Marker box_marker;
                box_marker.header.frame_id = "odom";
                box_marker.header.stamp = ros::Time::now();
                box_marker.id = ++id;
                box_marker.action = visualization_msgs::Marker::ADD;
                box_marker.type = visualization_msgs::Marker::LINE_STRIP;
                box_marker.color.r = 1.0;
                box_marker.color.g = 1.0;
                box_marker.color.b = 0.0;
                box_marker.color.a = 1.0;
                box_marker.scale.x = 0.08;
                vec_E<Vec2f> vertices;
                semantic_vehicle.second.vehicle.RetVehicleVertices(&vertices);
                for (int i = 0; i <= vertices.size(); ++i) {
                    geometry_msgs::Point point;
                    point.x = vertices[i % vertices.size()](0);
                    point.y = vertices[i % vertices.size()](1);
                    box_marker.points.push_back(point);
                }
                marker_array.markers.push_back(box_marker);
                // 显示信息
                visualization_msgs::Marker text_marker;
                text_marker.header.frame_id= "odom";
                text_marker.header.stamp = ros::Time::now();
                text_marker.action = visualization_msgs::Marker::ADD;
                text_marker.pose.orientation.w = 1.0;
                text_marker.id = ++id;
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.scale.z = 1.0;
                text_marker.color.b = 1;
                text_marker.color.g = 0;
                text_marker.color.r = 0;
                text_marker.color.a = 1;
                text_marker.text= "\nid: " +  std::to_string(semantic_vehicle.second.vehicle.id()) + "\nlane id: " + std::to_string(semantic_vehicle.second.nearest_lane_id) + "\nvel: " + std::to_string(semantic_vehicle.second.vehicle.state().velocity) + "\narc: " + std::to_string(semantic_vehicle.second.arc_len_onlane) + "\ndist: " + std::to_string(semantic_vehicle.second.dist_to_lane);
                text_marker.pose.position.x = semantic_vehicle.second.vehicle.Ret3DofState()(0);
                text_marker.pose.position.y = semantic_vehicle.second.vehicle.Ret3DofState()(1);
                marker_array.markers.push_back(text_marker);
            }
            this->debug_semantic_vehicle_vis_.publish(marker_array);
        }
        return semantic_vehicle_set;
    }

    // 道路信息转换
    SemanticLaneSet updateSemanticLaneSet(const State &raw_ego_vehicle_state, const std::vector<simulator::Lane> &lanes) const {
        SemanticLaneSet semantic_lane_set;
        for (auto lane: lanes) {
            SemanticLane semantic_lane;
            semantic_lane.id = lane.id;
            semantic_lane.l_lane_id = lane.left_id;
            if (semantic_lane.l_lane_id == kInvalidLaneId) {
                semantic_lane.l_change_avbl = false;
            } else {
                semantic_lane.l_change_avbl = true;
            }
            semantic_lane.r_lane_id = lane.right_id;
            if (semantic_lane.r_lane_id == kInvalidLaneId) {
                semantic_lane.r_change_avbl = false;
            } else {
                semantic_lane.r_change_avbl = true;
            }
            vec_Vecf<LaneDim> raw_samples;
            // 进行坐标转换
            for (auto point: lane.center) {
                Vecf<LaneDim> sample_point;
                sample_point(0) = point.x;
                sample_point(1) = point.y;
                double dist = sqrt(std::pow(raw_ego_vehicle_state.vec_position(0) - sample_point(0), 2.0) + std::pow(raw_ego_vehicle_state.vec_position(1) - sample_point(1), 2.0));
                // if (dist < 1.5 * this->range_) {
                //     raw_samples.push_back(sample_point);
                // }
                raw_samples.push_back(sample_point);
            }
            // 进行采样
            vec_Vecf<LaneDim> samples;
            for (double index = 0; static_cast<int>(index) < raw_samples.size(); index += static_cast<double>(raw_samples.size() - 1) / 6.0) {
                // std::cout << "raw size: " << raw_samples.size() << ", index: " << static_cast<int>((index)) << std::endl;
                samples.push_back(raw_samples.at(static_cast<int>((index))));
            }
            // 进行插值
            if (GetLaneBySamplePoints(samples, &semantic_lane.lane) != ErrorType::kSuccess) {
                std::cout << "generate lane failed" << std::endl;
                throw;
            }
            semantic_lane.length = semantic_lane.lane.end() - semantic_lane.lane.begin();
            semantic_lane_set.semantic_lanes.insert(std::make_pair(semantic_lane.id, semantic_lane));
        }
        // 进行可视化debug
        {
            // 删除之前的可视化
            visualization_msgs::MarkerArray delete_marker_array;
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "odom";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.action = visualization_msgs::Marker::DELETEALL;
            delete_marker.id = 0;
            delete_marker_array.markers.push_back(delete_marker);
            this->debug_semantic_lane_vis_.publish(delete_marker_array);
            visualization_msgs::MarkerArray marker_array;
            int id = 0;
            for(const auto &semantic_lane: semantic_lane_set.semantic_lanes) {
                // 显示中线
                visualization_msgs::Marker center_marker;
                center_marker.header.frame_id = "odom";
                center_marker.header.stamp = ros::Time::now();
                center_marker.id = ++id;
                center_marker.action = visualization_msgs::Marker::ADD;
                center_marker.type = visualization_msgs::Marker::LINE_STRIP;
                center_marker.color.r = 0.0;
                center_marker.color.g = 1.0;
                center_marker.color.b = 0.0;
                center_marker.color.a = 0.5;
                center_marker.scale.x = 0.05;
                vec_E<Vecf<2>> samples;
                double accum_dist;
                SampleLane(semantic_lane.second.lane, 0, semantic_lane.second.length, 1.0, &samples, &accum_dist);
                for (auto sample: samples) {
                    geometry_msgs::Point point;
                    point.x = sample(0);
                    point.y = sample(1);
                    center_marker.points.push_back(point);
                }
                marker_array.markers.push_back(center_marker);
                // 显示id
                visualization_msgs::Marker id_marker;
                id_marker.header.frame_id= "odom";
                id_marker.header.stamp = ros::Time::now();
                id_marker.action = visualization_msgs::Marker::ADD;
                id_marker.pose.orientation.w = 1.0;
                id_marker.id = ++id;
                id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                id_marker.scale.z = 2.0;
                id_marker.color.b = 0;
                id_marker.color.g = 0;
                id_marker.color.r = 1;
                id_marker.color.a = 1;
                id_marker.text= std::to_string(semantic_lane.second.id);
                id_marker.pose.position = center_marker.points.front();
                marker_array.markers.push_back(id_marker);
            }
            this->debug_semantic_lane_vis_.publish(marker_array);
        }
        return semantic_lane_set;
    }

    // 给出车辆状态，计算离车辆最经的道路id
    ErrorType GetNearestLaneIdUsingState(const Vec3f &state, const SemanticLaneSet &semantic_lane_set, int *id, double *distance, double *arc_len, double *angle_diff, double nearest_lane_range=2.0) const {
        // tuple: dist, arc_len, angle_diff, id
        std::set<std::tuple<double, double, double, int>, Comparer> lanes_in_dist;
        for (const auto &semantic_lane: semantic_lane_set.semantic_lanes) {
            std::tuple<double, double, double> dist_to_lane;
            if (GetDistanceToLanesUsing3DofState(state, semantic_lane.second.lane, dist_to_lane) != kSuccess) {
                return kWrongStatus;
            }
            lanes_in_dist.insert(std::tuple<double, double, double, int>(std::get<0>(dist_to_lane), std::get<1>(dist_to_lane), std::get<2>(dist_to_lane), semantic_lane.second.id));
        }

        if (lanes_in_dist.empty()) {
            printf("[GetNearestLaneIdUsingState]No nearest lane found.\n");
            return kWrongStatus;
        }
        
        // Otherwise, use the nearest lane
        *id = std::get<3>(*lanes_in_dist.begin());
        *distance = std::get<0>(*lanes_in_dist.begin());
        *arc_len = std::get<1>(*lanes_in_dist.begin());
        *angle_diff = std::get<2>(*lanes_in_dist.begin());
        return kSuccess;
    }
};

#endif  // LOCAL_PLANNING_HPP_
