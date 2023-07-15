#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

def compressed_image_callback(msg):
    try:
        # Convert the compressed image to raw image using cv_bridge
        bridge = CvBridge()
        raw_image = bridge.compressed_imgmsg_to_cv2(msg)

        # Create a new Image message and populate it with the raw image data
        raw_image_msg = bridge.cv2_to_imgmsg(raw_image, encoding="bgr8")

        # Publish the raw image on the topic "alphasense_driver_ros/cam0/image_raw"
        raw_image_pub.publish(raw_image_msg)
    except Exception as e:
        rospy.logerr("Error processing the compressed image: %s", str(e))

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("compressed_to_raw_image_converter")

    # Create a subscriber to listen to the compressed image topic
    compressed_image_sub = rospy.Subscriber("alphasense_driver_ros/cam0/compressed", CompressedImage, compressed_image_callback)

    # Create a publisher to publish the raw image on the desired topic
    raw_image_pub = rospy.Publisher("alphasense_driver_ros/cam0/image_raw", Image, queue_size=10)

    # Spin the node to receive and process messages
    rospy.spin()
