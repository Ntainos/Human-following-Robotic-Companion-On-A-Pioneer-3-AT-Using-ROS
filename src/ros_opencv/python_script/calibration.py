#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import math 
import os

home_dir = os.path.expanduser("~")
prototxt_path_human = os.path.join(home_dir, 'catkin_ws/src/ros_opencv/python_script/models/MobileNetSSD_deploy.prototxt')
model_path_human = os.path.join(home_dir, 'catkin_ws/src/ros_opencv/python_script/models/MobileNetSSD_deploy.caffemodel')
min_confidence_human = 0.5


# Define the camera feedback callback
def camera_callback(message):
    global detected_persons, frame
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(message)

    detected_persons = detect_person(frame)


def detect_person(frame):

    human_net = cv2.dnn.readNetFromCaffe(prototxt_path_human, model_path_human)

    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)

    human_net.setInput(blob)

    detections = human_net.forward()

    persons = []
    global upper_left_x , lower_right_x , upper_left_y , lower_right_y

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]

        # If the detection is a human and confidence is above a threshold
        if confidence > min_confidence_human and int(detections[0, 0, i, 1]) == 15:  # 15 = 'person' class
            box = detections[0, 0, i, 3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
            (upper_left_x , lower_right_x ,upper_left_y , lower_right_y) = box.astype("float")

            person_center = (upper_left_x + lower_right_x) // 2, (upper_left_y + lower_right_y) // 2

            persons.append(person_center)

    return persons

def visualservo(upper_left_x, upper_left_y, lower_right_x, lower_right_y):
    # Extract desired values
    desired_upper_left_x = 220.24078369140625 #194.939  
    desired_upper_left_y = 412.03704833984375 #428.092  
    desired_lower_right_x = 2.0499372482299805 #-2.656  
    desired_lower_right_y = 362.42591857910156 #377.354  
    desired_lower_left_x = desired_upper_left_x
    desired_lower_left_y = desired_lower_right_y
    desired_upper_right_x = desired_lower_right_x 
    desired_upper_right_y = desired_upper_left_y

        # Calculate measured values
    measured_upper_left_x = upper_left_x
    measured_upper_left_y = upper_left_y
    measured_lower_right_x = lower_right_x
    measured_lower_right_y = lower_right_y
    measured_lower_left_x = measured_upper_left_x
    measured_lower_left_y = measured_lower_right_y
    measured_upper_right_x = measured_lower_right_x 
    measured_upper_right_y = measured_upper_left_y
    # Extract camera parameters
    fx = 568.6158447265625 #584.2754532342447   
    fy = 589.6334228515625 #575.2132780735776
    cu = 415.8726894083811 #393.0282683248274
    cv = 198.55353781915983 #205.97329574599007
    # Extract robot parameters
    max_linear_velocity = 0.35
    max_angular_velocity = 0.15
    scaling_factor = 0.5  # Adjust this value based on your robot's capabilities

    print("measured_upper_left_x:", measured_upper_left_x)
    print("measured_upper_left_y:", measured_upper_left_y)
    print("measured_lower_right_x:", measured_lower_right_x)
    print("measured_lower_right_y:", measured_lower_right_y)


    # Vector of measured image coordinates
    m_t = np.array([measured_upper_left_x, measured_upper_left_y, measured_lower_right_x, measured_lower_right_y,
                    measured_lower_left_x, measured_lower_left_y, measured_upper_right_x, measured_upper_right_y])

    # Vector of desired values
    s_star = np.array([desired_upper_left_x, desired_upper_left_y, desired_lower_right_x, desired_lower_right_y,
                    desired_lower_left_x, desired_lower_left_y, desired_upper_right_x, desired_upper_right_y])

    # Calculate the error term
    e_t = m_t - s_star

    # Define the variables
    Z = 1.2
    u = (measured_upper_left_x + measured_lower_right_x)//2
    v = (measured_upper_left_y + measured_lower_right_y)//2

    # Calculate x and y based on the given expressions
    x = (u - cu) / fx
    y = (v - cv) / fy           


    print("u:", u)
    print("v:", v)
    print("x:", x)
    print("y:", y)
    # Lx matrixes
    L = np.array([
        [1/Z, 0, x/Z , x*y , -(1+np.power(x, 2)) , y ], 
        [0, -1/Z, y/Z , 1+np.power(y, 2) , -x*y , -x ],
        [1/Z, 0, x/Z , x*y , -(1+np.power(x, 2)) , y ], 
        [0, -1/Z, y/Z , 1+np.power(y, 2) , -x*y , -x ], 
        [1/Z, 0, x/Z , x*y , -(1+np.power(x, 2)) , y ], 
        [0, -1/Z, y/Z , 1+np.power(y, 2) , -x*y , -x ], 
        [1/Z, 0, x/Z , x*y , -(1+np.power(x, 2)) , y ], 
        [0, -1/Z, y/Z , 1+np.power(y, 2) , -x*y , -x ],  
    ])

    # Calculate the pseudo-inverse
    pseudoinverse = np.linalg.pinv(L)
    lambda_hat = 0.325
    vc = -lambda_hat * np.dot(pseudoinverse, e_t)

    print("Error term (e_t):", e_t)
    print("Calculated vc:", vc)

    # Assign values to move_cmd
    move_cmd = Twist()
    move_cmd.linear.x = -float(vc[0])  # Linear velocity in the x-direction (ahead)
    move_cmd.linear.y = 0.0            # Linear velocity in the y-direction (left)
    move_cmd.linear.z = 0.0            # Linear velocity in the z-direction (up)
    move_cmd.angular.x = 0.0           # Angular velocity around the x-axis
    move_cmd.angular.y = 0.0           # Angular velocity around the y-axis
    move_cmd.angular.z = float(vc[5])  # Angular velocity around the z-axis

    # Scale down linear and angular velocities
    move_cmd.linear.x *= scaling_factor
    move_cmd.angular.z *= scaling_factor

    # Ensure velocities are within limits
    move_cmd.linear.x = max(min(move_cmd.linear.x, max_linear_velocity), -max_linear_velocity)
    move_cmd.angular.z = max(min(move_cmd.angular.z, max_angular_velocity), -max_angular_velocity)

    print("Final linear.x:", move_cmd.linear.x)
    print("Final angular.z:", move_cmd.angular.z)

    return move_cmd

# Main function
if __name__ == "__main__":
    rospy.init_node("person_follower")

    detected_persons = []
    frame = None

    camera_subscriber = rospy.Subscriber("video_topic", Image, camera_callback)

    robot_publisher = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=10)
    move_cmd = Twist()

    rate = rospy.Rate(8)
    

    while not rospy.is_shutdown():
        while detected_persons:

                # Visualize the detected person
            #cv2.circle(frame, person_position, 10, (255, 0, 0), -1)
            move_cmd = visualservo(upper_left_x,upper_left_y,lower_right_x,lower_right_y)
            robot_publisher.publish(move_cmd)

            rate.sleep()

        # Stop the robot if no person is detected
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        robot_publisher.publish(move_cmd)

    rate.sleep()    


