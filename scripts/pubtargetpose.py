#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf

def publish_target_pose():
    rospy.init_node('target_pose_publisher')
    pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
    
    # Đảm bảo ROS có đủ thời gian để thiết lập publisher trước khi gửi
    rospy.sleep(1)

    pose = PoseStamped()
    pose.header.frame_id = "world"  # hoặc "map" tùy theo hệ thống
    pose.header.stamp = rospy.Time.now()

    # Vị trí mong muốn
    pose.pose.position.x = -0.5
    pose.pose.position.y = -0.6
    pose.pose.position.z = 0.9

    # Góc mong muốn (roll, pitch, yaw) → quaternion
    roll = 0.0
    pitch = 0.0
    yaw = 1.57  # ~90 độ
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    rospy.loginfo("Gửi /target_pose ")
    
    # Cập nhật timestamp và gửi một lần
    pose.header.stamp = rospy.Time.now()  # cập nhật timestamp trước khi gửi
    pub.publish(pose)
    
    # Đảm bảo rằng thông điệp đã được gửi trước khi kết thúc
    rospy.sleep(0.1)  # đợi một chút trước khi tắt

if __name__ == '__main__':
    try:
        publish_target_pose()
    except rospy.ROSInterruptException:
        pass
