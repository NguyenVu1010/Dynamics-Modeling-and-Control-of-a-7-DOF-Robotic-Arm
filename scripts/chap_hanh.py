#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

Kp = [100.0, 200.0, 100.0, 200.0, 100.0, 50.0, 20.0]
Kd = [50.0,  50.0,  50.0,  50.0,  50.0, 50.0, 20.0]
Ki = [5.0,   5.0,   5.0,   5.0,   5.0,  3.0, 1.0]

joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

current_position = [0.0] * 7
current_velocity = [0.0] * 7
target_position = None
external_effort = [0.0] * 7
last_u_time = 0.0
integral_error = [0.0] * 7

def joint_cb(msg):
    for i, j in enumerate(joint_names):
        if j in msg.name:
            idx = msg.name.index(j)
            current_position[i] = msg.position[idx]
            if len(msg.velocity) > idx:
                current_velocity[i] = msg.velocity[idx]

def u_cb(msg):
    global external_effort, last_u_time
    if len(msg.data) >= 7:
        external_effort = msg.data[:7]
        last_u_time = time.time()

def main():
    global target_position, integral_error

    rospy.init_node("hold_position")
    rospy.Subscriber("/joint_states", JointState, joint_cb)
    rospy.Subscriber("/u_pub", Float64MultiArray, u_cb)
    pub = rospy.Publisher("/cobot_controller/command", Float64MultiArray, queue_size=1)

    rate = rospy.Rate(300)
    timeout_duration = 0.3
    last_active_state = False
    last_time = time.time()

    while not rospy.is_shutdown():
        now = time.time()
        dt = now - last_time
        last_time = now

        active = (now - last_u_time) < timeout_duration

        if active:
            pub.publish(Float64MultiArray(data=external_effort))
            target_position = current_position.copy()
            integral_error = [0.0] * 7  # reset tích phân khi có điều khiển ngoài
            last_active_state = True
        else:
            if target_position is None or last_active_state:
                #target_position = current_position.copy()
                target_position = [0.0] * 7  # reset to zero if no external command
                rospy.loginfo_throttle(5, "[hold] No external command - holding current position.")
                integral_error = [0.0] * 7
                last_active_state = False

            effort = []
            for i in range(7):
                err = target_position[i] - current_position[i]
                derr = -current_velocity[i]
                integral_error[i] += err * dt

                u = Kp[i]*err + Kd[i]*derr + Ki[i]*integral_error[i]
                effort.append(u)

            pub.publish(Float64MultiArray(data=effort))

        rate.sleep()

if __name__ == "__main__":
    main()
