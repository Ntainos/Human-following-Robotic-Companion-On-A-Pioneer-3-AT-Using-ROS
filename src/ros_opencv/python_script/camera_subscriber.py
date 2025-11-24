#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from queue import Queue
import os

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'

home_dir = os.path.expanduser("~")
prototxt_path = os.path.join(home_dir, 'catkin_ws/src/ros_opencv/python_script/models/MobileNetSSD_deploy.prototxt')
model_path = os.path.join(home_dir, 'catkin_ws/src/ros_opencv/python_script/models/MobileNetSSD_deploy.caffemodel')

min_confidence = 0.5

classes = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle",
           "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
           "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

colors = np.random.uniform(0, 255, size=(len(classes), 3))

# Load model
net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)

# Shared queue to retrieve the latest captured frame
shared_queue = Queue(maxsize=1)

# Detected persons information
detected_persons = []
#corners = []

def detect_person(frame):
    global detected_persons

    height, width = frame.shape[0], frame.shape[1]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007, (300, 300), 130)

    net.setInput(blob)
    detected_objects = net.forward()

    frame_detected_persons = []  # List to store detected persons in the current frame

    for i in range(detected_objects.shape[2]):
        confidence = detected_objects[0, 0, i, 2]

        if confidence > min_confidence and int(detected_objects[0, 0, i, 1]) == 15:
            class_index = int(detected_objects[0, 0, i, 1])

            upper_left_x = int(detected_objects[0, 0, i, 3] * width)
            upper_left_y = int(detected_objects[0, 0, i, 4] * height)
            lower_right_x = int(detected_objects[0, 0, i, 5] * width)
            lower_right_y = int(detected_objects[0, 0, i, 6] * height)

            # Print the values
            print("Upper Left X:", upper_left_x)
            print("Upper Left Y:", upper_left_y)
            print("Lower Right X:", lower_right_x)
            print("Lower Right Y:", lower_right_y)

            person_label = f"{classes[class_index]}-{i}"  # Unique label for each detected person

            # Store information of the detected person
            frame_detected_persons.append({
                "label": person_label,
                "confidence": confidence,
                "position": ((upper_left_x + lower_right_x) // 2, (upper_left_y + lower_right_y) // 2)
            })


            #(upper_right_x, upper_right_y),(lower_left_x, lower_left_y)

            prediction_text = f"{person_label} : {confidence:.2f}%"
            cv2.rectangle(frame, (upper_left_x, upper_left_y), (lower_right_x, lower_right_y), colors[class_index], 3)
            cv2.putText(frame, prediction_text,
                        (upper_left_x, upper_left_y - 15 if upper_left_y > 30 else upper_left_y + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[class_index], 2)

    # Update the list of detected persons for the current frame
    detected_persons = frame_detected_persons

    cv2.imshow("Object Detection", frame)
    cv2.waitKey(3)

# Callback function for the camera feedback
def callbackFunction(message):
    global shared_queue
    bridgeObject = CvBridge()
    rospy.loginfo("Received a video message/frame")
    frame = bridgeObject.imgmsg_to_cv2(message)

    # Detect persons in the frame
    detect_person(frame)

# Subscribe to the video topic
rospy.init_node(subscriberNodeName, anonymous=True)
rospy.Subscriber(topicName, Image, callbackFunction)

rospy.spin()
cv2.destroyAllWindows()
