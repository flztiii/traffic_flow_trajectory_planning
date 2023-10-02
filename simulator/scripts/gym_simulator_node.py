#! /usr/bin/env python
#! -*- coding: utf-8 -*-

import highway_env
import gym
import rospy
import numpy as np
from simulator.srv import InitSimulator, StepSimulator, InitSimulatorRequest, StepSimulatorRequest, InitSimulatorResponse, StepSimulatorResponse
from simulator.msg import Lane, Vehicle
from geometry_msgs.msg import Point

class GymSimulation:
    def __init__(self) -> None:
        self.max_acc_ = 2.0
        self.max_steer_ = 0.7853981633974483
        self.lane_num_ = 4
        # 初始化环境
        highway_env.register_highway_envs()
        self.env_ = gym.make("highway-v0")
        self.env_.configure({
            "observation": {
                "type": "Kinematics",
                "vehicles_count": 50,
                "features": ["presence", "x", "y", "vx", "vy", "heading"],
                "features_range": {
                    "x": [-200, 200],
                    "y": [-10, 10],
                    "vx": [0, 30],
                    "vy": [-2, 2]
                },
                "absolute": True,
                "order": "sorted",
                "normalize": False,
                "clip": False,
                "see_behind": False,
                "observe_intentions": False,
            },
            "action": {
                "type": "ContinuousAction",
                "acceleration_range": (-self.max_acc_, self.max_acc_),
                "steering_range": (-self.max_steer_, self.max_steer_),
                "speed_range": (0, 32),
                "longitudinal": True,
                "lateral": True,
                "dynamical": False,
                "clip": True,
            },
            "lanes_count": self.lane_num_,
            "simulation_frequency": 5,
            "policy_frequency": 5,
            "show_trajectories": False,
            'duration': 480,
            'screen_height': 720,
            'screen_width': 1280,
        })
        # 初始化ros
        rospy.init_node("simulator_node")
        self.rosConnect()

    # 构建ros连接
    def rosConnect(self):
        # 初始化仿真服务
        init_simulator_server_name = rospy.get_param("~init_simulator_server_name")
        self.init_simulator_server_ = rospy.Service(init_simulator_server_name, InitSimulator, self.initSimulator)
        # 进行仿真服务
        step_simulator_server_name = rospy.get_param("~step_simulator_server_name")
        self.step_simulator_server_ = rospy.Service(step_simulator_server_name, StepSimulator, self.stepSimulator)

    # 初始化仿真
    def initSimulator(self, req: InitSimulatorRequest):
        obs, _ = self.env_.reset(seed=0)
        # 构建res
        res = InitSimulatorResponse()
        res.result = True
        res.start_timestamp = 0
        res.aim_lane_id = -1
        # 构建vehicle
        for i, ob in enumerate(obs):
            if ob[0] > 0:
                vehicle = Vehicle()
                vehicle.id = i
                vehicle.center = Point(ob[1], ob[2], 0)
                vehicle.width = 2.0
                vehicle.length = 5.0
                vehicle.orientation = ob[5]
                vehicle.speed = np.sqrt(ob[3]**2 + ob[4]**2)
                if i == 0:
                    res.ego_vehicle = vehicle
                else:
                    res.other_vehicles.append(vehicle)
        # 进行归一化
        for i in range(len(res.other_vehicles)):
            res.other_vehicles[i].center.x -= res.ego_vehicle.center.x
        res.ego_vehicle.center.x = 0
        # 进行道路获取
        idxs = range(self.lane_num_)
        for i in range(len(idxs)):
            lane = Lane()
            lane.id = idxs[i]
            if i > 0:
                lane.right_id = idxs[i - 1]
            else:
                lane.right_id = -1
            if i < len(idxs) - 1:
                lane.left_id = idxs[i + 1]
            else:
                lane.left_id = -1
            xs = np.linspace(res.ego_vehicle.center.x - 200, res.ego_vehicle.center.x + 200, 20)
            for x in xs:
                lane.center.append(Point(x, float(idxs[i]) * 4.0, 0))
                lane.left_boundary.append(Point(x, float(idxs[i]) * 4.0 + 2.0, 0))
                lane.right_boundary.append(Point(x, float(idxs[i]) * 4.0 - 2.0, 0))
            res.lanes.append(lane)
        return res

    # 进行仿真
    def stepSimulator(self, req: StepSimulatorRequest):
        # 进行更新
        action = (req.acc / self.max_acc_, req.steering / self.max_steer_)
        obs, _, done, truncated, info = self.env_.step(action)
        self.env_.render()
        # 得到结果
        res = StepSimulatorResponse()
        res.timestamp = 0
        if done or truncated:
            print(done, truncated, info)
            if truncated:
                res.done = -1
            elif info['crashed']:
                res.done = -1
            else:
                res.done = 1
        else:
            res.done = 0
        # 构建vehicle
        ego_vehicle_x = 0
        for i, ob in enumerate(obs):
            if ob[0] > 0:
                if i == 0:
                    ego_vehicle_x = ob[1]
                vehicle = Vehicle()
                vehicle.id = i
                vehicle.center = Point(ob[1], ob[2], 0)
                vehicle.width = 2.0
                vehicle.length = 5.0
                vehicle.orientation = ob[5]
                vehicle.speed = np.sqrt(ob[3]**2 + ob[4]**2)
                res.vehicles.append(vehicle)
        # 进行归一化
        for i in range(len(res.vehicles)):
            res.vehicles[i].center.x -= ego_vehicle_x
        return res

if __name__ == "__main__":
    gym_simualtor = GymSimulation()
    rospy.spin()