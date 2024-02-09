#! /usr/bin/env python3

import rospy, cv2, numpy, math, result2
from pathlib import Path
from datetime import date
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from time import strftime, localtime

greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
GREEN = (0, 255, 0)

image_bridge = CvBridge()
image = None
blur_kernel_size = 15
erode_iterations = 7
dilate_iterations = 60
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

plant_center_frame_position = {
    1: [4, 6, 1.1],
    2: [4, 6, 3.9],
    3: [4, 6, 6.7],
    4: [4, 13.5, 1.1],
    5: [4, 13.5, 3.9],
    6: [4, 13.5, 6.7],
    7: [4, 21, 1.1],
    8: [4, 21, 3.9],
    9: [4, 21, 6.7],
    10: [10, 6, 1.1],
    11: [10, 6, 3.9],
    12: [10, 6, 6.7],
    13: [10, 13.5, 1.1],
    14: [10, 13.5, 3.9],
    15: [10, 13.5, 6.7],
    16: [10, 21, 1.1],
    17: [10, 21, 3.9],
    18: [10, 21, 6.7],
    19: [16, 6, 1.1],
    20: [16, 6, 3.9],
    21: [16, 6, 6.7],
    22: [16, 13.5, 1.1],
    23: [16, 13.5, 3.9],
    24: [16, 13.5, 6.7],
    25: [16, 21, 1.1],
    26: [16, 21, 3.9],
    27: [16, 21, 6.7],
}
scan_status = False

roll, pitch, yaw = 0, 0, 0
position = None
trajectory_status = True

folder_name = f"{date.today()} {strftime('%H_%M_%S', localtime())}"

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

def filterDistance(plant_shelf_locations_with_yaw, plant_frames):
    filtered_plant_frames = {}
    for plant_shelf_location_with_yaw in plant_shelf_locations_with_yaw:
        required, minDist = None, 100000000000
        for drone_position in plant_frames.keys():
            current_distance = dist4D(plant_shelf_location_with_yaw, drone_position)
            if current_distance < minDist:
                required = drone_position
                minDist = current_distance
        filtered_plant_frames[tuple(plant_shelf_location_with_yaw)] = plant_frames[required]
    return filtered_plant_frames
def filterYaws(plant_frames_with_locations):
    plant_position_dictionary = {tuple(list(position)[:-1]): {} for position in plant_frames_with_locations.keys()}
    for position, plant_frames in plant_frames_with_locations.items():
        plant_position_dictionary[tuple(list(position)[:-1])][str(list(position)[-1])] = []
        for plant_frame in plant_frames.values():
            plant_position_dictionary[tuple(list(position)[:-1])][str(list(position)[-1])].append(plant_frame)
    return plant_position_dictionary
def filterPositions(plant_frames_with_locations):
    plant_positions = {}
    all_frames = 0
    for yaw_position in plant_frames_with_locations.values():
        current_shelf_plants = len(yaw_position["1.5707"]) if len(yaw_position["1.5707"]) < len(yaw_position["-1.5707"]) else len(yaw_position["-1.5707"])
        for frame_index in range(current_shelf_plants):
            plant_positions[all_frames] = [all_plant_frames[yaw_position["1.5707"][frame_index]], all_plant_frames[yaw_position["-1.5707"][current_shelf_plants-1-frame_index]]]
        all_frames += 1
    return plant_positions

if __name__ == "__main__":
    cwd = Path.cwd()
    folder = cwd / folder_name
    folder.mkdir(exist_ok=True)
    rospy.init_node("get_plant_frames")
    rate = rospy.Rate(10)
    image_subscriber = rospy.Subscriber("/red/camera/color/image_raw", Image, getImage)
    imu_subscriber = rospy.Subscriber("/red/imu", Imu, getImuData)
    position_subscriber = rospy.Subscriber("/red/pose", PoseStamped, getPosition)
    plant_locations = rospy.Subscriber("/red/plants_beds", String, getPlantLocationData)

    fruit_count_pub = rospy.Publisher("/fruit_count", Int32, queue_size=10) 

    while plant_location_data_raw_string == None:
        rate.sleep()
    plant_type = plant_location_data_raw_string.split(' ')[0]
    plant_shelf_location_indexes = [int(plant_location) for plant_location in plant_location_data_raw_string.split(' ')[1:]]
    plant_shelf_locations = [plant_center_frame_position[plant_shelf_location] for plant_shelf_location in plant_shelf_location_indexes]
    trajectory_subscriber = rospy.Subscriber("/in_trajectory", Bool, getTrajectoryStatus)
    scan_status = rospy.Subscriber("/scan", Bool, getScanStatus)
    while not rospy.is_shutdown() and trajectory_status:
        try:
            if scan_status:
                height, width = image.shape[:-1]
                rotation_angle = convertToDegree(math.pi - roll)
                rotation_matrix = cv2.getRotationMatrix2D((width//2, height//2), rotation_angle, 1)
                rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))
                blurred_image = cv2.GaussianBlur(rotated_image, (blur_kernel_size, blur_kernel_size), 0)
                hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv_image, greenLower, greenUpper)
                mask = cv2.erode(mask, None, iterations=erode_iterations)
                mask = cv2.dilate(mask, None, iterations=dilate_iterations)
                contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
                if len(contours) > 0:
                    position_key = tuple([position.pose.position.x, position.pose.position.y, position.pose.position.z, quaternionToRPY([position.pose.orientation.x, position.pose.orientation.y, position.pose.orientation.z, position.pose.orientation.w])[2]])
                plants_location, plants = [], []
                for contour in contours:
                    perimeter = cv2.arcLength(contour, True)
                    plant_location = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
                    plant_width, plant_height = plant_location[2][0][0]-plant_location[0][0][0], plant_location[2][0][1]-plant_location[0][0][1]
                    if abs(1-width/height) > square_tolerance:
                        continue
                    plants_location.append(plant_location)
                    plants.append(rotated_image[int(plants_location[-1][0][0][1]*(1+y_offset_percentage)):int(plants_location[-1][2][0][1]*(1+y_offset_percentage)), int(plants_location[-1][0][0][0]):int(plants_location[-1][2][0][0])])
                    if position_key not in plant_frames.keys():
                        plant_frames[position_key] = {}
                    plant_frames[position_key][plants_location[-1][0][0][0]] = len(all_plant_frames)
                    current_keys = list(plant_frames[position_key].keys())
                    current_keys.sort()
                    plant_frames[position_key] = {key: plant_frames[position_key][key] for key in current_keys}
                    all_plant_frames.append(plants[-1])
                for index, plant in enumerate(plants):
                    cv2.imshow(f"Plant {index+1}", plant)
                if prev_plant_count > len(plants):
                    [cv2.destroyWindow(f"Plant {plant_index+1}") for plant_index in range(len(plants), prev_plant_count)]
                prev_plant_count = len(plants)
                cv2.imshow("Mask", mask)
            else:
                rate.sleep()
            cv2.imshow("Camera", image)
        except Exception as exception:
            print(f"Warning: {exception}")
        cv2.waitKey(1)
    cv2.destroyAllWindows()
    plant_shelf_locations_with_yaw = [[plant_center_frame_position[plant_shelf_location][0], plant_center_frame_position[plant_shelf_location][1], plant_center_frame_position[plant_shelf_location][2], 1.570700] for plant_shelf_location in plant_shelf_location_indexes]
    plant_shelf_locations_with_yaw.extend([[plant_center_frame_position[plant_shelf_location][0], plant_center_frame_position[plant_shelf_location][1], plant_center_frame_position[plant_shelf_location][2], -1.570700] for plant_shelf_location in plant_shelf_location_indexes])
    plant_frames_with_locations = filterDistance(plant_shelf_locations_with_yaw, plant_frames)
    plant_frames_with_yaws = filterYaws(plant_frames_with_locations)
    plant_frames_with_indexes = filterPositions(plant_frames_with_yaws)
    fruit_count = result2.main(plant_frames_with_indexes, plant_type)

    print(fruit_count)
rate = rospy.Rate(10) # 10hz

fruit_count_msg = Int32()
fruit_count_msg.data = fruit_count
while not rospy.is_shutdown():
    fruit_count_pub.publish(fruit_count_msg)
    rate.sleep()