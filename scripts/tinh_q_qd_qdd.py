#!/usr/bin/env python3
import rospy
import json
import numpy as np
import os
from velocity.jacobi_t import JacobiT
from velocity.jacobi_r import JacobiR
from velocity.joint_limit_optimizer import JointLimitOptimizer
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
class CalQqdotQdd:
    def __init__(self, json_path="/home/nguyen1/seven_dof_ws/src/franka_h2/input/robot_parameters.json"):
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"File {json_path} không tồn tại.")
        
        with open(json_path, 'r') as f:
            params = json.load(f)

        self.d1 = params.get('d1', 0.0)
        self.d2 = params.get('d2', 0.0)
        self.d3 = params.get('d3', 0.0)
        self.d4 = params.get('d4', 0.0)
        self.d5 = params.get('d5', 0.0)
        self.d6 = params.get('d6', 0.0)
        self.d7 = params.get('d7', 0.0)
        self.a6 = params.get('a6', 0.0)
        self.g = params.get('g', 9.81)

        self.dt = 0.05
        self.K = 100 * np.eye(6) 
        self.err = np.zeros(6) 
        self.q0 = np.zeros(7)
        self.qd0 = np.zeros(7)
        self.q_current = np.zeros(7)
        self.rEvE = np.zeros(6)
        self.oriang = np.zeros(6)
        self.qd_prev = np.zeros(7)
        self.json_path = json_path

        self.target_pub = rospy.Publisher('/target_q_qd_qdd', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/trajectory_point', Float64MultiArray, self.trajectory_point_callback)
        rospy.Subscriber('/target_pose', PoseStamped, self.target_pose_cb)
    def target_pose_cb(self, msg):
        self.q0 = self.q_current.copy()
        rospy.loginfo(f"Đã nhận target_pose mới :{self.q0}")
    def joint_state_callback(self, msg):
        self.q_current = np.array(msg.position)
        self.qd0 = np.array(msg.velocity)

    def trajectory_point_callback(self, msg):
        if len(msg.data) < 12:
            rospy.logwarn("Dữ liệu trajectory_point không đủ 12 phần tử.")
            return

        self.rEvE = np.array(msg.data[0:6])
        self.oriang = np.array(msg.data[6:12])
        
        try:
            qdot = self.compute()
            q_target = self.integrate_qdot(qdot)
            qdd_target = self.derivative_qdot(qdot)
            self.qd_prev = qdot.copy()
            self.send_target(q_target, qdot, qdd_target)
        except Exception as e:
            rospy.logwarn(f"Lỗi trong trajectory_point_callback: {e}")


    def compute(self):
        q = self.q_current
        rEvE = self.rEvE
        oriang = self.oriang

        eta1dot = rEvE[3:6]  
        ome = oriang[3:6]  
        psi, theta, phi = oriang[0:3]  

        try:
            iQ = np.array([
                [np.cos(psi)*np.tan(theta), np.sin(psi)*np.tan(theta), 1.0],
                [-np.sin(psi), np.cos(psi), 0.0],
                [np.cos(psi)/np.cos(theta), np.sin(psi)/np.cos(theta), 0.0]
            ])
        except:
            iQ = np.eye(3)

        eta2dot = iQ @ ome  
        eta_dot = np.concatenate([eta1dot, eta2dot])  

        jacobi_t = JacobiT(json_path=self.json_path)
        jacobi_r = JacobiR()
        JT = jacobi_t.compute(q)
        JR = jacobi_r.compute(q)

        J = np.vstack([JT[:3, :], iQ @ JR])
        JJT = J @ J.T

        try:
            J_plus = J.T @ np.linalg.inv(JJT)
        except np.linalg.LinAlgError:
            J_plus = J.T @ np.linalg.pinv(JJT)

        z0_optimizer = JointLimitOptimizer(json_path=self.json_path)
        z0 = z0_optimizer.compute_z0(q)
        En = np.eye(7)

        qdot = J_plus @ (eta_dot + self.K @ self.err) + 10 * (En - J_plus @ J) @ z0
        
        return qdot

    def integrate_qdot(self, qdot):
        self.q0 += qdot * self.dt
        return self.q0.copy()

    def derivative_qdot(self, qdot):
        return (qdot - self.qd_prev)/self.dt

    def send_target(self, q_target, qd, qdd_target):
        msg = Float64MultiArray()
        target_data = np.concatenate((q_target, qd, qdd_target))
        msg.data = target_data.tolist()
        self.target_pub.publish(msg)
        rospy.loginfo_throttle(0.01, f"Đã gửi target q, qd, qdd: {msg.data}")

if __name__ == "__main__":
    try:
        rospy.init_node('joint_velocity_calculator', anonymous=False)
        CalQqdotQdd()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
