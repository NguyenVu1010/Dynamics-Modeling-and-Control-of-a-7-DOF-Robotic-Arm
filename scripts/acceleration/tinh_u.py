#!/usr/bin/env python3
import json
import numpy as np
import os
import rospy
from inertia_m import InertiaMatrix
from coriolis_c import CoriolisMatrix
from gravity_g import GravityMatrix
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

json_path = '/home/nguyen1/seven_dof_ws/src/franka_h2/input/robot_parameters.json'

def current_time_sec():
    return rospy.Time.now().to_sec()

class ControlInput:
    def __init__(self, json_path=json_path):
        if not os.path.exists(json_path):
            raise FileNotFoundError(f"File {json_path} không tồn tại.")
        with open(json_path, 'r') as f:
            params = json.load(f)
        # Khởi tạo các thành phần động lực
        self.inertia = InertiaMatrix(json_path)
        self.coriolis = CoriolisMatrix(json_path)
        self.gravity = GravityMatrix(json_path)

        # PD gains
        self.kp = np.diag(np.array([100.0, 200.0, 100.0, 200.0, 100.0, 50.0, 20.0])) 
        self.kd = np.diag(np.array([50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 20.0]))
        
        # Trạng thái robot
        self.qR = np.zeros(7)
        self.qdR = np.zeros(7)

        # Dữ liệu target mới nhất
        self.q = np.zeros(7)
        self.qdot = np.zeros(7)
        self.qddot = np.zeros(7)
        self.u_last = np.zeros(7)
        self.has_target = False
        self.last_target_time = 0.0

        # ROS interfaces
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/target_q_qd_qdd', Float64MultiArray, self.target_state_callback)
        self.u_pub = rospy.Publisher('/u_pub', Float64MultiArray, queue_size=1)
        # Timer để publish u_last ở 100Hz
        self.pub_timer = rospy.Timer(rospy.Duration(0.01), self.publish_timer)

        rospy.loginfo('[compute_u_node] Node tính lực điều khiển đã khởi động.')

    def joint_state_callback(self, msg):
        if len(msg.position) >= 7 and len(msg.velocity) >= 7:
            self.qR = np.array(msg.position[:7])
            self.qdR = np.array(msg.velocity[:7])

    def target_state_callback(self, msg):
        # Khi nhận target mới: tính u và lưu
        if len(msg.data) >= 21:
            self.q = np.array(msg.data[0:7])
            self.qdot = np.array(msg.data[7:14])
            self.qddot = np.array(msg.data[14:21])
            self.has_target = True
            self.last_target_time = current_time_sec()
            
            try:
                pos_error = self.q - self.qR       # shape: (7,)
                vel_error = self.qdot - self.qdR   # shape: (7,)
                # Tính y = PD + feedforward
                y = self.kp @ pos_error + self.kd @ vel_error + self.qddot
                # Tính M, C, G
                data = np.load('/home/nguyen1/seven_dof_ws/src/cobot30_5/data/dynamics_matrices.npz')
                M = data['M']
                C = data['C']
                G = data['G']

                # Tính u = M*y + C*qdR + G
                u = (M @ y.reshape(7,1) + C @ self.qdR.reshape(7,1) + G.reshape(7,1)).flatten()
                self.u_last = u.flatten() 
                
            except Exception as e:
                rospy.logerr(f'[compute_u_node] Lỗi khi tính toán u: {e}')
        else:
            rospy.logwarn('[compute_u_node] target_q_qd_qdd thiếu dữ liệu (>=21 phần tử).')

    def publish_timer(self, event):
        # Dừng gửi nếu không có target mới trong 0.3s
        if not self.has_target or (current_time_sec() - self.last_target_time) > 0.02:
            return
        msg = Float64MultiArray(data=self.u_last.tolist())
        self.u_pub.publish(msg)
        rospy.loginfo_throttle(0.01, f'[compute_u_node] Updated u_last = {self.u_last.tolist()}')

if __name__ == '__main__':
    rospy.init_node('compute_u_node', anonymous=True)
    ControlInput()
    rospy.spin()