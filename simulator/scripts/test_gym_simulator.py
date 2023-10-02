#!/usr/bin/env python
#!-*- coding: utf-8 -*-

import rospy
import numpy as np
from simulator.msg import Lane, Vehicle
from simulator.srv import InitSimulator, InitSimulatorRequest, InitSimulatorResponse
from simulator.srv import StepSimulator, StepSimulatorRequest, StepSimulatorResponse
from visualization_msgs.msg import Marker, MarkerArray

# 可视化道路
def visulizeLanes(lanes: list[Lane]):
    marker_array = MarkerArray()
    id = 0
    for lane in lanes:
        center_marker = Marker()
        center_marker.header.frame_id = "odom"
        center_marker.header.stamp = rospy.Time.now()
        center_marker.id = id
        id += 1
        center_marker.action = Marker().ADD
        center_marker.type = Marker().LINE_STRIP
        center_marker.color.r = 1.0
        center_marker.color.g = 0.0
        center_marker.color.b = 0.0
        center_marker.color.a = 0.5
        center_marker.scale.x = 0.05
        center_marker.points = lane.center
        marker_array.markers.append(center_marker)
        if lane.left_id == -1:
            left_marker = Marker()
            left_marker.header.frame_id = "odom"
            left_marker.header.stamp = rospy.Time.now()
            left_marker.id = id
            id += 1
            left_marker.action = Marker().ADD
            left_marker.type = Marker().LINE_STRIP
            left_marker.color.r = 1.0
            left_marker.color.g = 1.0
            left_marker.color.b = 1.0
            left_marker.color.a = 1.0
            left_marker.scale.x = 0.15
            left_marker.points = lane.left_boundary
            marker_array.markers.append(left_marker)
        elif lane.right_id == -1:
            right_marker = Marker()
            right_marker.header.frame_id = "odom"
            right_marker.header.stamp = rospy.Time.now()
            right_marker.id = id
            id += 1
            right_marker.action = Marker().ADD
            right_marker.type = Marker().LINE_STRIP
            right_marker.color.r = 1.0
            right_marker.color.g = 1.0
            right_marker.color.b = 1.0
            right_marker.color.a = 1.0
            right_marker.scale.x = 0.15
            right_marker.points = lane.right_boundary
            marker_array.markers.append(right_marker)
        else:
            left_marker = Marker()
            left_marker.header.frame_id = "odom"
            left_marker.header.stamp = rospy.Time.now()
            left_marker.id = id
            id += 1
            left_marker.action = Marker().ADD
            left_marker.type = Marker().LINE_LIST
            left_marker.color.r = 1.0
            left_marker.color.g = 1.0
            left_marker.color.b = 1.0
            left_marker.color.a = 1.0
            left_marker.scale.x = 0.15
            left_marker.points = lane.left_boundary[:len(lane.left_boundary) - len(lane.left_boundary)%2]
            marker_array.markers.append(left_marker)
            right_marker = Marker()
            right_marker.header.frame_id = "odom"
            right_marker.header.stamp = rospy.Time.now()
            right_marker.id = id
            id += 1
            right_marker.action = Marker().ADD
            right_marker.type = Marker().LINE_LIST
            right_marker.color.r = 1.0
            right_marker.color.g = 1.0
            right_marker.color.b = 1.0
            right_marker.color.a = 1.0
            right_marker.scale.x = 0.15
            right_marker.points = lane.right_boundary[:len(lane.right_boundary) - len(lane.right_boundary)%2]
            marker_array.markers.append(right_marker)
    return marker_array

# 可视化车辆
def visualizeVehicle(vehicle: Vehicle, color: tuple):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.id = vehicle.id
    marker.action = Marker().ADD
    marker.type = Marker().CUBE
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.scale.x = vehicle.length
    marker.scale.y = vehicle.width
    marker.scale.z = 1.5
    marker.pose.position = vehicle.center
    marker.pose.orientation.z = np.sin(0.5 * vehicle.orientation)
    marker.pose.orientation.w = np.cos(0.5 * vehicle.orientation)
    return marker

if __name__ == "__main__":
    # 初始化ros
    rospy.init_node("test_simulator_node")
    rospy.wait_for_service("/simulator/init_simulator")
    init_simulator_service = rospy.ServiceProxy("/simulator/init_simulator", InitSimulator, True)
    rospy.wait_for_service("/simulator/step_simulator")
    step_simulator_service = rospy.ServiceProxy("/simulator/step_simulator", StepSimulator, True)
    lane_visualize_pub = rospy.Publisher("/simulator/lane_visualize", MarkerArray, queue_size=10)
    ego_vehicle_visualize_pub = rospy.Publisher("/simulator/ego_vehicle_visualize", MarkerArray, queue_size=10)
    scenario_visualize_pub = rospy.Publisher("/simulator/scenario_visualize", MarkerArray, queue_size=10)

    # 初始化simulator
    input("input to start simulator test")
    init_simulator_request = InitSimulatorRequest()
    while not rospy.is_shutdown():
        try:
            init_simulator_response = init_simulator_service.call(init_simulator_request)
            if init_simulator_response.result:
                break
        except rospy.ServiceException:
            print("init simulator failed")
            exit(0)
    # 得到信息
    ego_vehicle = init_simulator_response.ego_vehicle
    ego_vehicle_id = init_simulator_response.ego_vehicle.id
    vehicles = init_simulator_response.other_vehicles
    lanes = init_simulator_response.lanes
    print("init simulator finished, ego vehicle id: ", ego_vehicle_id)
    # 可视化道路
    lane_visualize_pub.publish(visulizeLanes(lanes))
    # 清除之前的可视化
    delete_marker_array = MarkerArray()
    delete_marker = Marker()
    delete_marker.header.frame_id = "odom"
    delete_marker.header.stamp = rospy.Time.now()
    delete_marker.action = Marker().DELETEALL
    delete_marker.id = 0
    delete_marker_array.markers.append(delete_marker)
    # 可视化自身车辆
    ego_vehicle_visualize_pub.publish(delete_marker_array)
    ego_vehicle_marker_array = MarkerArray()
    ego_vehicle_marker_array.markers.append(visualizeVehicle(ego_vehicle, (0, 1, 0, 1)))
    ego_vehicle_visualize_pub.publish(ego_vehicle_marker_array)
    # 可视化场景
    scenario_visualize_pub.publish(delete_marker_array)
    scenario_marker_array = MarkerArray()
    for vehicle in vehicles:
        scenario_marker_array.markers.append(visualizeVehicle(vehicle, (1, 0, 0, 1)))
    scenario_visualize_pub.publish(scenario_marker_array)
    # 开始进行模拟
    while True:
        input()
        try:
            step_simulator_request = StepSimulatorRequest(0.0, 0.0)
            step_simulator_response = step_simulator_service.call(step_simulator_request)
        except rospy.ServiceException:
            print("step simulator failed")
            exit(0)
        # print("simulation current timestamp: ", step_simulator_response.timestamp)
        vehicles = step_simulator_response.vehicles
        # 找到自身车辆
        find_ego_vehicle = False
        for vehicle in vehicles:
            if vehicle.id == ego_vehicle_id:
                ego_vehicle = vehicle
                find_ego_vehicle = True
                break
        if not find_ego_vehicle:
            print("ego vehicle not exist anymore")
            exit(0)
        vehicles.remove(ego_vehicle)
        # 清除之前的可视化
        delete_marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = "odom"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.action = Marker().DELETEALL
        delete_marker.id = 0
        delete_marker_array.markers.append(delete_marker)
        # 可视化自身车辆
        ego_vehicle_visualize_pub.publish(delete_marker_array)
        ego_vehicle_marker_array = MarkerArray()
        ego_vehicle_marker_array.markers.append(visualizeVehicle(ego_vehicle, (0, 1, 0, 1)))
        ego_vehicle_visualize_pub.publish(ego_vehicle_marker_array)
        # 可视化场景
        scenario_visualize_pub.publish(delete_marker_array)
        scenario_marker_array = MarkerArray()
        for vehicle in vehicles:
            scenario_marker_array.markers.append(visualizeVehicle(vehicle, (1, 0, 0, 1)))
        scenario_visualize_pub.publish(scenario_marker_array)
        if step_simulator_response.done != 0:
            break