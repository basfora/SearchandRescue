#!/usr/bin/env python
import rospy

# Because of transformations
import tf
from nav_msgs.msg import Odometry


def callback(msg):
    t = geometry_msgs.msg.TransformStamped()

    p = msg.pose.pose.position
    q = msg.pose.pose.orientation

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = p.x
    t.transform.translation.y = p.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    odom_broadcaster.sendTransform(t)

    # next, we'll publish the odometry message over ROS
    odom_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('fake_odometry')
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    rospy.Subscriber('/ground_truth/state', Odometry, callback)
    odom_broadcaster = tf.TransformBroadcaster()
    rospy.spin()
