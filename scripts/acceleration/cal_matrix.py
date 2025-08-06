#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from inertia_m import InertiaMatrix
from coriolis_c import CoriolisMatrix
from gravity_g import GravityMatrix
json_path = '/home/nguyen1/seven_dof_ws/src/franka_h2/input/robot_parameters.json'
class MatrixCalculator:
    def __init__(self):
        rospy.init_node('cal_matrix_node', anonymous=True)

        self.qR = np.zeros(7)
        self.qdR = np.zeros(7)
        self.got_state = False

        # Load các module động học
        self.inertia = InertiaMatrix(json_path)
        self.coriolis = CoriolisMatrix(json_path)
        self.gravity = GravityMatrix(json_path)

        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Timer(rospy.Duration(0.01), self.compute_and_save)  # 10Hz

        rospy.loginfo('[cal_matrix_node] Node tính toán M, C, G đã khởi động.')

    def joint_state_callback(self, msg):
        if len(msg.position) >= 7 and len(msg.velocity) >= 7:
            self.qR = np.array(msg.position[:7])
            self.qdR = np.array(msg.velocity[:7])
            self.got_state = True
        
    def compute_and_save(self, event):
        if not self.got_state:
            return

        try:
            M = self.inertia.compute(self.qR)
            C = self.coriolis.compute(self.qR, self.qdR)
            G = self.gravity.compute(self.qR)
            
            # Lưu ra file npz
            np.savez('/home/nguyen1/seven_dof_ws/src/cobot30_5/data/dynamics_matrices.npz', M=M, C=C, G=G)
        except Exception as e:
            rospy.logerr(f'[cal_matrix_node] Lỗi khi tính M, C, G: {e}')


if __name__ == '__main__':
    try:
        node = MatrixCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
