#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

publisherNodeName = 'camera_sensor_publisher'
topicName = 'video_topic'

rospy.init_node(publisherNodeName, anonymous=True)

publisher = rospy.Publisher(topicName, Image, queue_size=60)
rate = rospy.Rate(8)
cap = cv2.VideoCapture(2)
bridgeObject = CvBridge()

while not rospy.is_shutdown():
    returnValue, image = cap.read()
    if returnValue:
        rospy.loginfo('Video frame captured and published')
        imageToTransmit = bridgeObject.cv2_to_imgmsg(image)
        publisher.publish(imageToTransmit)
    rate.sleep()

