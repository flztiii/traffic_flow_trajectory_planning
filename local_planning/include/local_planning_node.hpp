#ifndef LOCAL_PLANNING_NODE_HPP_
#define LOCAL_PLANNING_NODE_HPP_

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <memory>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "simulator/InitSimulator.h"
#include "simulator/StepSimulator.h"
#include "simulator/Lane.h"
#include "simulator/Vehicle.h"
#include "local_planner.hpp"
#include "lane.hpp"
#include "vehicle.hpp"
#include "local_planner.hpp"

// 局部规划节点
class LocalPlanningNode {
 public:
    // 构造函数
    LocalPlanningNode () {
        // 获取ros句柄
        this->nh_ = ros::NodeHandle("~");
        // 加载参数
        this->loadParams();
        // 初始化ros相关消息服务
        this->initRosConnection();
        std::cout << "ros initialized" << std::endl;
        // 开启ros线程
        std::thread ros_thread = std::thread(&LocalPlanningNode::rosConnector, this);
        ros_thread.detach();
        std::cout << "ros connected" << std::endl;
        // 初始化规划器
        this->local_planner_ptr_ = std::make_shared<LocalPlanner>();
    };

    // 析构函数
    ~LocalPlanningNode () {};

    // 规划函数
    ErrorType startPlanningOnce(int test_count=0) {
        // 首先初始化仿真场景
        simulator::InitSimulator::Request init_simulator_request;
        simulator::InitSimulator::Response init_simulator_response;
        init_simulator_request.simulator_hz = this->simulator_hz_;
        init_simulator_request.simulator_max_steps = this->simulator_max_steps_;
        while (true) {
            if (!this->init_simulator_service_.call(init_simulator_request, init_simulator_response)) {
                std::cout << "call init simulator service failed" << std::endl;
                throw;
            }
            if (init_simulator_response.result) {
                break;
            }
        }
        // 得到初始信息
        double start_timestamp = init_simulator_response.start_timestamp;
        int ego_vehicle_id = init_simulator_response.ego_vehicle.id;
        std::vector<simulator::Lane> raw_lanes = init_simulator_response.lanes;
        int aim_lane_id = init_simulator_response.aim_lane_id;
        std::cout << "init simulator finished, start timestamp: " << start_timestamp << std::endl;
        // 可视化道路
        this->visualizeLanes(raw_lanes);
        // 自身车辆设置
        VehicleParam ego_vehicle_param = VehicleParam();
        State ego_vehicle_state;
        ego_vehicle_state.time_stamp = start_timestamp;
        ego_vehicle_state.vec_position(0) = init_simulator_response.ego_vehicle.center.x - ego_vehicle_param.d_cr() * cos(init_simulator_response.ego_vehicle.orientation);
        ego_vehicle_state.vec_position(1) = init_simulator_response.ego_vehicle.center.y - ego_vehicle_param.d_cr() * sin(init_simulator_response.ego_vehicle.orientation);
        ego_vehicle_state.angle = init_simulator_response.ego_vehicle.orientation;
        ego_vehicle_state.velocity = init_simulator_response.ego_vehicle.speed;
        this->lock_.lock();
        this->ego_vehicle_ = Vehicle(ego_vehicle_id, ego_vehicle_param, ego_vehicle_state);
        this->lock_.unlock();
        // 控制信息设置
        VehicleControlSignal control_signal = VehicleControlSignal(0.0, 0.0);
        // 开始进行模拟
        for (int step = 0; step < this->simulator_max_steps_; step++) {
            // 前向模拟情景
            simulator::StepSimulator::Request step_simulator_request;
            simulator::StepSimulator::Response step_simulator_response;
            if (!this->step_simulator_service_.call(step_simulator_request, step_simulator_response)) {
                std::cout << "call step simulator service failed" << std::endl;
                throw;
            }
            // 得到当前信息
            double current_timestamp = step_simulator_response.timestamp;
            std::vector<simulator::Vehicle> other_vehicles;
            simulator::Vehicle driver_vehicle;
            for (auto vehicle: step_simulator_response.vehicles) {
                if (vehicle.id != ego_vehicle_id) {
                    other_vehicles.push_back(vehicle);
                } else {
                    driver_vehicle = vehicle;
                }
            }
            // 当前情景可视化
            this->visualizeEgoVehicle(this->ego_vehicle_);
            this->visualizeDriver(driver_vehicle);
            this->visualizeScenario(other_vehicles);
            // 记录车辆的速度，加速度等信息
            std::string file_path = ros::package::getPath("local_planning") + "/record/state_record_" + std::to_string(test_count) + ".csv";
            std::ofstream data_file(file_path, std::ios::out|std::ios::app);
            data_file << this->ego_vehicle_.state().vec_position[0] << "," << this->ego_vehicle_.state().vec_position[1] << "," << this->ego_vehicle_.state().angle << "," << this->ego_vehicle_.state().curvature << "," << this->ego_vehicle_.state().velocity << "," << this->ego_vehicle_.state().acceleration << "\n";
            data_file.close();
            // 进行规划
            auto start_time = ros::WallTime::now();
            ErrorType planning_result = this->local_planner_ptr_->planOnce(this->ego_vehicle_, raw_lanes, other_vehicles, aim_lane_id, this->planning_horizon_, control_signal);
            auto end_time = ros::WallTime::now();
            std::cout << "time consuming: " << (end_time - start_time).toNSec() / 1e6 << " ms" << std::endl;
            if (planning_result != kSuccess) {
                std::cout << "planning failed, test failed" << std::endl;
                return kWrongStatus;
            }
            // 车辆前向模拟
            std::cout << "step " << step << " finished" << std::endl;
            this->lock_.lock();
            this->ego_vehicle_.set_state(control_signal.state);
            this->lock_.unlock();
            // getchar();
        }
        // 测试完成
        return kSuccess;
    };

    // 规划函数
    ErrorType startGymPlanningOnce(int test_count=0) {
        // 首先初始化仿真场景
        simulator::InitSimulator::Request init_simulator_request;
        simulator::InitSimulator::Response init_simulator_response;
        init_simulator_request.simulator_hz = this->simulator_hz_;
        init_simulator_request.simulator_max_steps = this->simulator_max_steps_;
        while (true) {
            if (!this->init_simulator_service_.call(init_simulator_request, init_simulator_response)) {
                std::cout << "call init simulator service failed" << std::endl;
                throw;
            }
            if (init_simulator_response.result) {
                break;
            }
        }
        // 得到初始信息
        double start_timestamp = init_simulator_response.start_timestamp;
        int ego_vehicle_id = init_simulator_response.ego_vehicle.id;
        std::vector<simulator::Lane> raw_lanes = init_simulator_response.lanes;
        std::vector<simulator::Vehicle> other_vehicles = init_simulator_response.other_vehicles;
        simulator::Vehicle driver_vehicle = init_simulator_response.ego_vehicle;
        int aim_lane_id = init_simulator_response.aim_lane_id;
        std::cout << "init simulator finished, start timestamp: " << start_timestamp << std::endl;
        // 可视化道路
        this->visualizeLanes(raw_lanes);
        // 自身车辆设置
        VehicleParam ego_vehicle_param = VehicleParam();
        State ego_vehicle_state;
        ego_vehicle_state.time_stamp = start_timestamp;
        ego_vehicle_state.vec_position(0) = driver_vehicle.center.x - ego_vehicle_param.d_cr() * cos(driver_vehicle.orientation);
        ego_vehicle_state.vec_position(1) = driver_vehicle.center.y - ego_vehicle_param.d_cr() * sin(driver_vehicle.orientation);
        ego_vehicle_state.angle = driver_vehicle.orientation;
        ego_vehicle_state.velocity = driver_vehicle.speed;
        this->lock_.lock();
        this->ego_vehicle_ = Vehicle(ego_vehicle_id, ego_vehicle_param, ego_vehicle_state);
        this->lock_.unlock();
        // 控制信息设置
        VehicleControlSignal control_signal = VehicleControlSignal(0.0, 0.0);
        // 开始进行模拟
        while (true) {
            // 当前情景可视化
            this->visualizeEgoVehicle(this->ego_vehicle_);
            this->visualizeDriver(driver_vehicle);
            this->visualizeScenario(other_vehicles);
            // 记录车辆的速度，加速度等信息
            std::string file_path = ros::package::getPath("local_planning") + "/record/state_record_" + std::to_string(test_count) + ".csv";
            std::ofstream data_file(file_path, std::ios::out|std::ios::app);
            data_file << this->ego_vehicle_.state().vec_position[0] << "," << this->ego_vehicle_.state().vec_position[1] << "," << this->ego_vehicle_.state().angle << "," << this->ego_vehicle_.state().curvature << "," << this->ego_vehicle_.state().velocity << "," << this->ego_vehicle_.state().acceleration << "\n";
            data_file.close();
            // 进行规划
            auto start_time = ros::WallTime::now();
            ErrorType planning_result = this->local_planner_ptr_->planOnce(this->ego_vehicle_, raw_lanes, other_vehicles, aim_lane_id, this->planning_horizon_, control_signal);
            auto end_time = ros::WallTime::now();
            std::cout << "time consuming: " << (end_time - start_time).toNSec() / 1e6 << " ms" << std::endl;
            if (planning_result != kSuccess) {
                std::cout << "planning failed, test failed" << std::endl;
                return kWrongStatus;
            }
            // 进行场景更新
            simulator::StepSimulator::Request step_simulator_request;
            step_simulator_request.acc = control_signal.state.acceleration;
            step_simulator_request.steering = atan(5.0 * control_signal.state.curvature);
            simulator::StepSimulator::Response step_simulator_response;
            if (!this->step_simulator_service_.call(step_simulator_request, step_simulator_response)) {
                std::cout << "call step simulator service failed" << std::endl;
                throw;
            }
            // 更新当前信息
            other_vehicles.clear();
            for (auto vehicle: step_simulator_response.vehicles) {
                if (vehicle.id != ego_vehicle_id) {
                    other_vehicles.push_back(vehicle);
                } else {
                    driver_vehicle = vehicle;
                }
            }
            // 判断结果
            if (step_simulator_response.done == 1) {
                break;
            } else if (step_simulator_response.done == -1) {
                return kWrongStatus;
            }
            // 车辆前向模拟
            ego_vehicle_state = control_signal.state;
            ego_vehicle_state.time_stamp = start_timestamp;
            ego_vehicle_state.vec_position(0) = driver_vehicle.center.x - ego_vehicle_param.d_cr() * cos(driver_vehicle.orientation);
            ego_vehicle_state.vec_position(1) = driver_vehicle.center.y - ego_vehicle_param.d_cr() * sin(driver_vehicle.orientation);
            ego_vehicle_state.angle = driver_vehicle.orientation;
            ego_vehicle_state.velocity = driver_vehicle.speed;
            this->lock_.lock();
            this->ego_vehicle_.set_state(ego_vehicle_state);
            this->lock_.unlock();
            std::cout << "expected_state: " << std::endl;
            control_signal.state.print();
            std::cout << "new state: " << std::endl;
            ego_vehicle_state.print();
            // getchar();
        }
        // 测试完成
        return kSuccess;
    };

 private:
    ros::NodeHandle nh_;
    ros::ServiceClient init_simulator_service_;
    ros::ServiceClient step_simulator_service_;
    ros::Publisher lane_visualize_pub_;
    ros::Publisher ego_vehicle_visualize_pub_;
    ros::Publisher driver_vehicle_visualize_pub_;
    ros::Publisher scenario_visualize_pub_;
    int simulator_hz_;
    int simulator_max_steps_;
    double planning_horizon_;
    std::shared_ptr<LocalPlanner> local_planner_ptr_;
    Vehicle ego_vehicle_;
    std::mutex lock_;
    tf::TransformBroadcaster br_;  // 位置播放

    // 可视化本车
    void visualizeEgoVehicle(const Vehicle &ego_vehicle) const {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 0.3;
        marker.scale.x = ego_vehicle.param().length();
        marker.scale.y = ego_vehicle.param().width();
        marker.scale.z = 0.01;
        marker.pose.position.x = ego_vehicle.state().vec_position(0) + cos(ego_vehicle.state().angle) * ego_vehicle.param().d_cr();
        marker.pose.position.y = ego_vehicle.state().vec_position(1) + sin(ego_vehicle.state().angle) * ego_vehicle.param().d_cr();;
        marker.pose.orientation.z = sin(0.5 * ego_vehicle.state().angle);
        marker.pose.orientation.w = cos(0.5 * ego_vehicle.state().angle);
        marker_array.markers.push_back(marker);
        // 显示信息
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id= "odom";
        text_marker.header.stamp = ros::Time::now();
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.orientation.w = 1.0;
        text_marker.id = 1;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.scale.z = 1.0;
        text_marker.color.b = 1;
        text_marker.color.g = 0;
        text_marker.color.r = 0;
        text_marker.color.a = 1;
        text_marker.text= "vel: " + std::to_string(ego_vehicle.state().velocity);
        text_marker.pose.position.x = ego_vehicle.state().vec_position(0) + cos(ego_vehicle.state().angle) * ego_vehicle.param().d_cr();
        text_marker.pose.position.y = ego_vehicle.state().vec_position(1) + sin(ego_vehicle.state().angle) * ego_vehicle.param().d_cr();;
        marker_array.markers.push_back(text_marker);
        this->ego_vehicle_visualize_pub_.publish(marker_array);
    }

    // 可视化司机
    void visualizeDriver(const simulator::Vehicle &vehicle) const {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 0.3;
        marker.scale.x = vehicle.length;
        marker.scale.y = vehicle.width;
        marker.scale.z = 0.01;
        marker.pose.position = vehicle.center;
        marker.pose.orientation.z = sin(0.5 * vehicle.orientation);
        marker.pose.orientation.w = cos(0.5 * vehicle.orientation);
        marker_array.markers.push_back(marker);
        this->driver_vehicle_visualize_pub_.publish(marker_array);
    }

    // 可视化场景
    void visualizeScenario(const std::vector<simulator::Vehicle> &vehicles) const {
        // 删除之前的可视化
        visualization_msgs::MarkerArray marker_array, delete_marker_array;
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = "odom";
        delete_marker.header.stamp = ros::Time::now();
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        delete_marker.id = 0;
        delete_marker_array.markers.push_back(delete_marker);
        this->scenario_visualize_pub_.publish(delete_marker_array);
        // 进行可视化
        for (auto vehicle: vehicles) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time::now();
            marker.id = vehicle.id;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.color.a = 0.3;
            marker.scale.x = vehicle.length;
            marker.scale.y = vehicle.width;
            marker.scale.z = 0.01;
            marker.pose.position = vehicle.center;
            marker.pose.orientation.z = sin(0.5 * vehicle.orientation);
            marker.pose.orientation.w = cos(0.5 * vehicle.orientation);
            marker_array.markers.push_back(marker);
        }
        this->scenario_visualize_pub_.publish(marker_array);
    }

    // 可视化道路信息
    void visualizeLanes(const std::vector<simulator::Lane> &lanes) const {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        for (auto lane: lanes) {
            visualization_msgs::Marker center_marker;
            center_marker.header.frame_id = "odom";
            center_marker.header.stamp = ros::Time::now();
            center_marker.id = id;
            id += 1;
            center_marker.action = visualization_msgs::Marker::ADD;
            center_marker.type = visualization_msgs::Marker::LINE_STRIP;
            center_marker.color.r = 1.0;
            center_marker.color.g = 0.0;
            center_marker.color.b = 0.0;
            center_marker.color.a = 0.3;
            center_marker.scale.x = 0.03;
            center_marker.points = lane.center;
            marker_array.markers.push_back(center_marker);
            if (lane.left_id == -1) {
                visualization_msgs::Marker left_marker;
                left_marker.header.frame_id = "odom";
                left_marker.header.stamp = ros::Time::now();
                left_marker.id = id;
                id += 1;
                left_marker.action = visualization_msgs::Marker::ADD;
                left_marker.type =visualization_msgs::Marker::LINE_STRIP;
                left_marker.color.r = 1.0;
                left_marker.color.g = 1.0;
                left_marker.color.b = 1.0;
                left_marker.color.a = 1.0;
                left_marker.scale.x = 0.15;
                left_marker.points = lane.left_boundary;
                marker_array.markers.push_back(left_marker);
            } else if (lane.right_id == -1) {
                visualization_msgs::Marker right_marker;
                right_marker.header.frame_id = "odom";
                right_marker.header.stamp = ros::Time::now();
                right_marker.id = id;
                id += 1;
                right_marker.action = visualization_msgs::Marker::ADD;
                right_marker.type = visualization_msgs::Marker::LINE_STRIP;
                right_marker.color.r = 1.0;
                right_marker.color.g = 1.0;
                right_marker.color.b = 1.0;
                right_marker.color.a = 1.0;
                right_marker.scale.x = 0.15;
                right_marker.points = lane.right_boundary;
                marker_array.markers.push_back(right_marker);
            } else {
                visualization_msgs::Marker left_marker;
                left_marker.header.frame_id = "odom";
                left_marker.header.stamp = ros::Time::now();
                left_marker.id = id;
                id += 1;
                left_marker.action = visualization_msgs::Marker::ADD;
                left_marker.type = visualization_msgs::Marker::LINE_LIST;
                left_marker.color.r = 1.0;
                left_marker.color.g = 1.0;
                left_marker.color.b = 1.0;
                left_marker.color.a = 1.0;
                left_marker.scale.x = 0.15;
                left_marker.points = std::vector<geometry_msgs::Point>(lane.left_boundary.begin(), lane.left_boundary.begin() + lane.left_boundary.size() - lane.left_boundary.size()%2);
                marker_array.markers.push_back(left_marker);
                visualization_msgs::Marker right_marker;
                right_marker.header.frame_id = "odom";
                right_marker.header.stamp = ros::Time::now();
                right_marker.id = id;
                id += 1;
                right_marker.action = visualization_msgs::Marker::ADD;
                right_marker.type = visualization_msgs::Marker::LINE_LIST;
                right_marker.color.r = 1.0;
                right_marker.color.g = 1.0;
                right_marker.color.b = 1.0;
                right_marker.color.a = 1.0;
                right_marker.scale.x = 0.15;
                right_marker.points = std::vector<geometry_msgs::Point>(lane.right_boundary.begin(), lane.right_boundary.begin() + lane.right_boundary.size() - lane.right_boundary.size()%2);
                marker_array.markers.push_back(right_marker);
            }
        }
        this->lane_visualize_pub_.publish(marker_array);
    }

    void loadParams() {
        this->nh_.getParam("simulator_hz", this->simulator_hz_);
        this->nh_.getParam("simulator_max_steps", this->simulator_max_steps_);
        this->nh_.getParam("planning_horizon", this->planning_horizon_);
    }

    // 初始化连接ros
    void initRosConnection() {
        // 初始化仿真服务
        std::string init_simulator_service_name;
        this->nh_.getParam("init_simulator_service_name", init_simulator_service_name);
        ros::service::waitForService(init_simulator_service_name);
        this->init_simulator_service_ = this->nh_.serviceClient<simulator::InitSimulator>(init_simulator_service_name, true);
        // 进行仿真服务
        std::string step_simulator_service_name;
        this->nh_.getParam("step_simulator_service_name", step_simulator_service_name);
        ros::service::waitForService(step_simulator_service_name);
        this->step_simulator_service_ = this->nh_.serviceClient<simulator::StepSimulator>(step_simulator_service_name, true);
        // 可视化发布topic
        this->lane_visualize_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/local_planning/lane_visualize", 10);
        this->ego_vehicle_visualize_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/local_planning/ego_vehicle_visualize", 10);
        this->driver_vehicle_visualize_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/local_planning/driver_vehicle_visualize", 10);
        this->scenario_visualize_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/local_planning/scenario_visualize", 10);
    }

    // 连接ros
    void rosConnector() {
        ros::Rate loop_rate(100);
        while (ros::ok()) {
            ros::spinOnce();
            tf::Transform transform;
            this->lock_.lock();
            transform.setOrigin(tf::Vector3(this->ego_vehicle_.state().vec_position(0), this->ego_vehicle_.state().vec_position(1), 0.0));
            tf::Quaternion q;
            q.setRPY(0, 0, this->ego_vehicle_.state().angle);
            transform.setRotation(q);
            this->lock_.unlock();
            this->br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
            loop_rate.sleep();
        }
    }
};

#endif  // LOCAL_PLANNING_NODE_HPP_
