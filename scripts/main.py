#! /usr/bin/env python3

import rospy, cv2, numpy, math
from pathlib import Path
from datetime import date
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from time import strftime, localtime, sleep

from app import run_detection

# global variables
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
GREEN = (0, 255, 0)

image_bridge = CvBridge()
image = None
depth = None
mask = None
image_count = 0
blur_kernel_size = 15
erode_iterations = 6
dilate_iterations = 61
all_plant_frames = []
plant_frames = {}
prev_plant_count = 0
plant_location_data_raw_string = None
plant_type = None
plant_shelf_indexes = None
square_tolerance = 1
y_offset_percentage = -0.1
y_increase_value = 0.1
x_increase_value = 0.1
distance_from_shelf = 3
distance_tolerance = 0.5
required_frame_count = 1
delay = 2 

scan_status = False

roll, pitch, yaw = 0, 0, 0
position = None
trajectory_status = True



# for logging
folder_name = f"{date.today()} {strftime('%H_%M_%S', localtime())}"



# helper functions
def convertToDegree(radian):
    return radian * 180 / math.pi
def convertToRadian(degree):
    return degree * math.pi / 180
def quaternionToRPY(quaternion):
    x, y, z, w = quaternion
    roll = numpy.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = numpy.arcsin(2 * (w * y - z * x))
    yaw = numpy.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return roll, pitch, yaw
def dist4D(position_1, position_2):
    return math.sqrt((position_1[0]-position_2[0])**2+(position_1[1]-position_2[1])**2+(position_1[2]-position_2[2])**2+(position_1[3]-position_2[3])**2)

def getImage(ros_image):
    global image
    image = image_bridge.imgmsg_to_cv2(ros_image)
def getDepth(ros_depth):
    global depth, mask
    depth = image_bridge.imgmsg_to_cv2(ros_depth, desired_encoding="passthrough")
    mask = cv2.inRange(depth, distance_from_shelf-distance_tolerance, distance_from_shelf+distance_tolerance)
def getImuData(imu_data):
    global roll, pitch, yaw
    roll, pitch, yaw = quaternionToRPY([imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w])
def getPosition(ros_position):
    global position
    position = ros_position

def getTrajectoryStatus(trajectory_status_ros):
    global trajectory_status
    trajectory_status = trajectory_status_ros.data
def getScanStatus(scan_status_ros):
    global scan_status
    scan_status = scan_status_ros.data
def getPlantLocationData(plant_location_data_string):
    global plant_location_data_raw_string
    plant_location_data_raw_string = plant_location_data_string.data


# main function
if __name__ == "__main__":
    rospy.init_node("get_plant_frames")
    rate = rospy.Rate(10)

    image_subscriber = rospy.Subscriber("/red/camera/color/image_raw", Image, getImage)
    depth_subscriber = rospy.Subscriber("/red/camera/depth/image_raw", Image, getDepth)
    imu_subscriber = rospy.Subscriber("/red/imu", Imu, getImuData)
    position_subscriber = rospy.Subscriber("/red/pose", PoseStamped, getPosition)
    plant_locations = rospy.Subscriber("/red/plants_beds", String, getPlantLocationData)
    trajectory_subscriber = rospy.Subscriber("/in_trajectory", Bool, getTrajectoryStatus)
    scan_status = rospy.Subscriber("/scan", Bool, getScanStatus)
    

    #wait for plant_beds topic
    while plant_location_data_raw_string == None:
        rospy.loginfo("Waiting for /plants_beds topic!")
        rate.sleep()
    plant_type = plant_location_data_raw_string.split(' ')[0]
    plant_shelf_location_indexes = [int(plant_location) for plant_location in plant_location_data_raw_string.split(' ')[1:]]

    image_count = 0
    frame_count = 0
    while not rospy.is_shutdown() and trajectory_status:
        try:
            filtered_image = cv2.bitwise_and(image, image, mask=mask)
            cv2.imshow("Filtered", filtered_image)


            if scan_status:
                if frame_count == 0:
                    sleep(delay)
                    frame_count += 1
                    continue
                if frame_count > required_frame_count:
                    continue
                
                filtered_image = cv2.bitwise_and(image, image, mask=mask)
                
                height, width = filtered_image.shape[:2]
                detection_img, centerX, centerY = run_detection(filtered_image, depth)

                cv2.imshow("Detection Output", detection_img)
                image_count += 1
            else:
                frame_count = 0
                pass

        except Exception as exception:
            print(f"Warning: {exception}")
        cv2.waitKey(1)
    cv2.destroyAllWindows()
    print()