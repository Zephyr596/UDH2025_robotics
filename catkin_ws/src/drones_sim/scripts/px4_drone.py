#!/usr/bin/env python

"""
PX4无人机控制节点
该脚本实现了一个基于MAVROS的PX4无人机控制类，提供起飞、降落、航点导航等功能
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome
from tf.transformations import *

from sensor_msgs.msg import NavSatFix

from math import *
import numpy as np
from numpy.linalg import norm
import time
import os
import json

from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointSetCurrent
from geographic_msgs.msg import GeoPointStamped

class Drone:
    def __init__(self):
        """无人机类初始化"""
        # 获取无人机名称参数
        self.uav_name = str(rospy.get_param(rospy.get_name() + '/drone'))
        rospy.loginfo("INIT-" + self.uav_name + "-DRONE")
        
        # 初始化状态变量
        self.pose = None  # 当前位姿
        self.yaw = 0      # 当前偏航角
        self.sp = None    # 目标位置
        self.hz = 10      # 控制频率
        self.rate = rospy.Rate(self.hz)  # 控制循环频率

        # MAVROS状态相关
        self.current_state = State()
        self.global_pose = None
        self.prev_request = None
        self.prev_state = None
        self.state = None

        # 原点设置相关
        self.set_origin_once = True
        self.origin_position = None

        # 初始化发布器
        self.setpoint_publisher = rospy.Publisher('/uav' + self.uav_name + '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.set_gp_origin_pub = rospy.Publisher('/uav' + self.uav_name + '/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)

        # 初始化服务客户端
        self.arming_client = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/cmd/takeoff', CommandTOL)

        # 航点相关服务
        self.push_waypoints = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/mission/push', WaypointPush)
        self.clear_waypoints = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/mission/clear', WaypointClear)
        self.set_current_waypoint = rospy.ServiceProxy('/uav' + self.uav_name + '/mavros/mission/set_current', WaypointSetCurrent)        

        # 初始化订阅器
        rospy.Subscriber('/uav' + self.uav_name + '/mavros/state', State, self.state_callback)
        rospy.Subscriber('/uav' + self.uav_name + '/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
        rospy.Subscriber('/uav' + self.uav_name + '/mavros/global_position/global', NavSatFix, self.gps_callback)

        # 任务文件路径
        self.pathname_task_list = "/home/sim/UDH2025_robotics/catkin_ws/src/drones_sim/mission/"
        self.filename_task_list = 'task_list_ASAD' + self.uav_name + '.json'

        # 读取任务文件
        self.mission_path = self.read_mission_json()

    def read_mission_json(self):
        """从JSON文件读取任务路径
        
        返回:
            mission_list (list[CTask, ...]): 包含任务路径的列表
        """       
        print("file path and name:", self.pathname_task_list + self.filename_task_list)

        mission_paths = []

        # 检查文件是否存在
        if os.path.isfile(self.pathname_task_list + self.filename_task_list):
            with open(self.pathname_task_list + self.filename_task_list) as json_file:
                data = json.load(json_file)

            # 解析任务数据
            for task in data['task_list']:
                if int(task['code']) == 10:  # 10表示航点任务
                    path_task = []
                    for pose in task['poses']:
                        path_task.append([float(pose['lat']), float(pose['lon']), float(pose['alt'])])
                    mission_paths.append(path_task)

            # 记录任务信息
            rospy.logwarn("---Mission readed---")
            rospy.loginfo("n_tasks: " + str(len(mission_paths)))
            rospy.loginfo("Task id and code")
            
            for path in mission_paths:
                print(path)
        else:
            rospy.logwarn("---Mission not readed!!! Task list is not exist ---")

        return mission_paths
    
    def gps_callback(self, data):
        """GPS数据回调函数"""
        self.global_pose = data

        # 设置原点位置（仅执行一次）
        if self.set_origin_once and self.global_pose != None:
            self.origin_position = data
            self.set_origin(self.origin_position)
            self.set_origin_once = False
            
    def state_callback(self, state):
        """状态回调函数"""
        self.current_state = state

    def drone_pose_callback(self, pose_msg):
        """位姿回调函数"""
        self.pose = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])

    def arm(self):
        """解锁无人机"""
        # 发送初始设置点
        for i in range(self.hz):
            self.publish_setpoint([0,0,-1])
            self.rate.sleep()
    
        # 等待飞控连接
        while not self.current_state.connected:
            print('Waiting for FCU connection...')
            self.rate.sleep()

        # 切换到OFFBOARD模式并解锁
        prev_request = rospy.get_time()
        prev_state = self.current_state
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.current_state.mode != "OFFBOARD" and (now - prev_request > 2.):
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                prev_request = now 
            else:
                if not self.current_state.armed and (now - prev_request > 2.):
                   self.arming_client(True)
                   prev_request = now 

            # 打印状态变化
            if prev_state.armed != self.current_state.armed:
                print("Vehicle armed: %r" % self.current_state.armed)
            if prev_state.mode != self.current_state.mode: 
                print("Current mode: %s" % self.current_state.mode)
            prev_state = self.current_state

            if self.current_state.armed:
                break
            # 持续发送设置点
            self.publish_setpoint([0,0,-1])
            self.rate.sleep()

    @staticmethod
    def get_setpoint(x, y, z, yaw=np.pi/2):
        """生成设置点消息
        
        参数:
            x, y, z: 目标位置
            yaw: 目标偏航角
            
        返回:
            set_pose: PoseStamped消息
        """
        set_pose = PoseStamped()
        set_pose.pose.position.x = x
        set_pose.pose.position.y = y
        set_pose.pose.position.z = z
        q = quaternion_from_euler(0, 0, yaw)
        set_pose.pose.orientation.x = q[0]
        set_pose.pose.orientation.y = q[1]
        set_pose.pose.orientation.z = q[2]
        set_pose.pose.orientation.w = q[3]
        return set_pose
    
    def publish_setpoint(self, sp, yaw=np.pi/2):
        """发布设置点"""
        setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw)
        setpoint.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(setpoint)

    def takeoff(self, height):
        """起飞到指定高度"""
        print("Takeoff...")
        self.sp = self.pose
        while self.pose[2] < height:
            self.sp[2] += 0.07
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def set_origin(self, pose):
        """设置GPS原点"""
        # 等待发布器初始化
        rospy.sleep(1)

        origin = GeoPointStamped()
        origin.position.latitude = pose.latitude
        origin.position.longitude = pose.longitude
        origin.position.altitude = pose.altitude

        rospy.loginfo("Setting custom gp origin position...")
        self.set_gp_origin_pub.publish(origin)

    def hover(self, t_hold):
        """悬停指定时间"""
        print('Position holding...')
        t0 = time.time()
        self.sp = self.pose
        while not rospy.is_shutdown():
            t = time.time()
            if t - t0 > t_hold and t_hold > 0: break
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def land(self):
        """降落"""
        print("Landing...")
        self.sp = self.pose
        while self.sp[2] > - 1.0:
            self.sp[2] -= 0.08
            self.publish_setpoint(self.sp)
            self.rate.sleep()
        self.stop()

    def stop(self):
        """停止无人机"""
        while self.current_state.armed or self.current_state.mode == "OFFBOARD":
            if self.current_state.armed:
                self.arming_client(False)
            if self.current_state.mode == "OFFBOARD":
                self.set_mode_client(base_mode=0, custom_mode="MANUAL")
            self.rate.sleep()

    def goTo(self, wp, mode='global', tol=0.05):
        """前往指定位置
        
        参数:
            wp: 目标位置
            mode: 模式 ('global' 或 'relative')
            tol: 到达容差
        """
        if mode=='global':
            goal = wp
        elif mode=='relative':
            goal = self.pose + wp
        print("Going to a waypoint...")
        self.sp = self.pose
        while norm(goal - self.pose) > tol:
            n = (goal - self.sp) / norm(goal - self.sp)
            self.sp += 0.03 * n
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def set_mode(self, mode):
        """设置飞行模式"""
        if self.set_mode_client(custom_mode=mode).mode_sent:
            rospy.loginfo(f"{mode} mode enabled")
        else:
            rospy.logwarn(f"Failed to set {mode} mode!")

    def test(self):
        """测试函数"""
        self.arm()
        self.takeoff(height=5.0)
        self.send_gps_path()

    def send_gps_path(self):
        """发送GPS航点路径"""
        try:
            # 清除现有航点
            self.clear_waypoints()
            waypoints = []
            is_first = True

            # 添加任务航点
            for cord in self.mission_path[0]:
                wp = Waypoint()
                wp.frame = 3  # 全局坐标系
                wp.command = 16  # NAV_WAYPOINT命令
                wp.is_current = is_first  # 设置为第一个航点
                is_first = False
                wp.autocontinue = True
                wp.x_lat = cord[0]
                wp.y_long = cord[1]
                wp.z_alt = cord[2]
                last_cord = cord
                waypoints.append(wp)

            # 添加降落航点
            wp = Waypoint()
            wp.frame = 3  # 全局坐标系
            wp.command = 21  # NAV_LAND命令
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = last_cord[0]
            wp.y_long = last_cord[1]
            wp.z_alt = 0  # 高度（降落时忽略）
            waypoints.append(wp)

            # 推送航点到PX4
            response = self.push_waypoints(start_index=0, waypoints=waypoints)

            if response.success:
                rospy.loginfo("Waypoints sent successfully!")
                rospy.sleep(1)
                self.set_mode("AUTO.MISSION")
            else:
                rospy.logerr("Failed to send waypoints.")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('drone_control', anonymous=True)
    try:
        control = Drone() 
        control.test()

        while not rospy.is_shutdown():
            rospy.spin()  # 保持节点运行并处理回调

    except rospy.ROSInterruptException:
        pass