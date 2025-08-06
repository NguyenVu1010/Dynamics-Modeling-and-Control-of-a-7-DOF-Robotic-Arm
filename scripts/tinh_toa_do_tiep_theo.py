#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from trajectory.rectilinear_xt import RectilinearTrajectory
from trajectory.orientation_rt import OrientationTrajectory
import tf

current_pose = None
target_pose = None
new_target_received = False

def current_pose_cb(msg):
    global current_pose
    current_pose = msg

def target_pose_cb(msg):
    global target_pose, new_target_received
    target_pose = msg
    new_target_received = True  # Đánh dấu có target mới

def extract_xyz_rpy(pose_msg):
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    x, y, z = pos.x, pos.y, pos.z
    q = [ori.x, ori.y, ori.z, ori.w]
    rpy = tf.transformations.euler_from_quaternion(q)
    return [x, y, z], list(rpy)

def main():
    global current_pose, target_pose, new_target_received

    json_path = '/home/nguyen1/seven_dof_ws/src/franka_h2/input/robot_parameters.json'
    rospy.init_node('trajectory_node')
    pub = rospy.Publisher('/trajectory_point', Float64MultiArray, queue_size=10)

    rospy.Subscriber('/current_pose', PoseStamped, current_pose_cb)
    rospy.Subscriber('/target_pose', PoseStamped, target_pose_cb)

    rospy.loginfo("Đang chờ dữ liệu ban đầu từ /current_pose và /target_pose...")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and (current_pose is None or target_pose is None):
        rate.sleep()

    rospy.loginfo("Bắt đầu lắng nghe và sinh quỹ đạo nếu có target mới...")
    pub_rate = rospy.Rate(20)  # tốc độ xuất điểm quỹ đạo

    traj = None
    ori = None
    T = rospy.get_param("~duration", 10.0)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        if new_target_received:
            new_target_received = False

            pos_start, rpy_start = extract_xyz_rpy(current_pose)
            pos_goal, rpy_goal = extract_xyz_rpy(target_pose)

            traj = RectilinearTrajectory(json_file=json_path, start=pos_start, goal=pos_goal, T=T)
            ori = OrientationTrajectory(json_file=json_path, start_rpy=rpy_start, goal_rpy=rpy_goal, T=T)
            start_time = rospy.Time.now()

            rospy.loginfo(f"🎯 Đã nhận target mới, sinh lại quỹ đạo tới: {pos_goal}, RPY: {rpy_goal}")

        if traj is not None and ori is not None:
            t = (rospy.Time.now() - start_time).to_sec()
            if t <= T:
                rEvE = traj.compute_trajectory(t)
                phi = ori.compute_trajectory(t)

                msg = Float64MultiArray(data=list(rEvE[:3]) + list(rEvE[3:]) + list(phi[:3]) + list(phi[3:]))
                pub.publish(msg)
                rospy.loginfo_throttle(0.01, f"[{t:.2f}s] Pos: {rEvE[:3]}, RPY: {phi[:3]}")
            else:
                # Quỹ đạo hết, chờ target mới
                rospy.loginfo_throttle(5, "⏳ Đang chờ target mới...")
        else:
            rospy.loginfo_throttle(5, "⏳ Chưa có quỹ đạo, chờ dữ liệu...")

        pub_rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
