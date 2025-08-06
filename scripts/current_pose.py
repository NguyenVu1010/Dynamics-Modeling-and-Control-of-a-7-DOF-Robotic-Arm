#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def listen_to_tf():
    rospy.init_node('tf_listener')

    # Tạo listener để nghe từ topic /tf
    listener = tf.TransformListener()

    # Tạo publisher cho topic current_pose
    current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)

    # Đặt thời gian để chờ transform giữa hai frame
    rate = rospy.Rate(10.0)  # Tốc độ 10Hz
    while not rospy.is_shutdown():
        try:
            # Đợi transform giữa base_link và panda_link7
            listener.waitForTransform('base_link', 'link_7', rospy.Time(0), rospy.Duration(1.0))

            # Lấy transform từ base_link đến panda_link7 tại thời điểm hiện tại
            (trans, rot) = listener.lookupTransform('base_link', 'link_7', rospy.Time(0))

            # Tọa độ điểm cuối
            x, y, z = trans
            #rospy.loginfo("Tọa độ điểm cuối: x = %f, y = %f, z = %f", x, y, z)

            # Quay điểm cuối (quaternion)
            qx, qy, qz, qw = rot

            # Chuyển quaternion sang góc Euler
            euler = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])
            roll, pitch, yaw = euler

            #rospy.loginfo("Góc quay của điểm cuối: roll = %f, pitch = %f, yaw = %f", roll, pitch, yaw)

            # Tạo PoseStamped message
            pose_msg = PoseStamped()

            # Gán header cho message
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'base_link'  # Khung tham chiếu của điểm cuối

            # Gán tọa độ và góc quay vào PoseStamped
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            # Publish PoseStamped message lên topic current_pose
            current_pose_pub.publish(pose_msg)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Không thể lấy transform giữa base_link và link_7.")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        listen_to_tf()
    except rospy.ROSInterruptException:
        pass
