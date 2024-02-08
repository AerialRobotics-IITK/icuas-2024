#! /usr/bin/env python3

import rospy, cv2, numpy, math, pickle 
from pathlib import Path
from datetime import date
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from time import strftime, localtime

bridge = CvBridge()

greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

GREEN = (0, 255, 0)

y_offset_percentage = -0.1
y_increase_value = 0.1
x_increase_value = 0.1
square_tolerance = 0.2

prev_plant_count = 0

blur_kernel_size = 13

roll, pitch, yaw = 0, 0, 0
position = None
all_plant_frames = []
plant_frames = {}
trajectory_status = True

folder_name = f"{date.today()} {strftime('%H_%M_%S', localtime())}"
image_count = 1
width = 1000

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

def convertToDegree(radian):
    return radian * 180 / math.pi
def convertToRadian(degree):
    return degree * math.pi / 180

def doOffSet(point):
    point[1] = int(point[1] * (1 + y_offset_percentage))
    return point

def quaternionToRPY(quaternion):
    x, y, z, w = quaternion
    roll = numpy.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = numpy.arcsin(2 * (w * y - z * x))
    yaw = numpy.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return roll, pitch, yaw

def getTrajectoryStatus(trajectory_status_ros):
    global trajectory_status
    trajectory_status = trajectory_status_ros
def getImuData(imu_data):
    global roll, pitch, yaw
    roll, pitch, yaw = quaternionToRPY([imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w])
def getImage(ros_image):
    global image_count, prev_plant_count, all_plant_frames, width
    cv_image = bridge.imgmsg_to_cv2(ros_image)
    width = cv_image.shape[1]
    #cv2.imshow("Camera", cv_image)
    height, width = cv_image.shape[:2]
    rotation_angle = convertToDegree(math.pi - roll)
    rotation_matrix = cv2.getRotationMatrix2D((width//2, height//2), rotation_angle, 1)
    final_image = cv2.warpAffine(cv_image, rotation_matrix, (width, height))
    blurred_image = cv2.GaussianBlur(final_image, (blur_kernel_size, blur_kernel_size), 0)
    hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=7)
    mask = cv2.dilate(mask, None, iterations=70)
    #cv2.imshow("Mask", mask)
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        position_key = tuple([position.pose.position.x, position.pose.position.y, position.pose.position.z, quaternionToRPY([position.pose.orientation.x, position.pose.orientation.y, position.pose.orientation.z, position.pose.orientation.w])[2]])
    plants_location, plants = [], []
    rectangle_final_image = final_image.copy()
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        plant_location = cv2.approxPolyDP(contour, 0.05 * perimeter, True)
        width, height = plant_location[2][0][0]-plant_location[0][0][0], plant_location[2][0][1]-plant_location[0][0][1]
        if abs(1-width/height) > square_tolerance:
            continue
        plants_location.append(plant_location)
        plants.append(final_image[int(plants_location[-1][0][0][1]*(1+y_offset_percentage)):int(plants_location[-1][2][0][1]*(1+y_offset_percentage)), plants_location[-1][0][0][0]:plants_location[-1][2][0][0]])
        if position_key not in plant_frames.keys():
            plant_frames[position_key] = {}
        plant_frames[position_key][(tuple(doOffSet(plants_location[-1][0][0])), tuple(doOffSet(plants_location[-1][2][0])))] = len(all_plant_frames)
        all_plant_frames.append(plants[-1])
        #cv2.rectangle(rectangle_final_image, doOffSet(plants_location[-1][0][0]), doOffSet(plants_location[-1][2][0]), GREEN, 2)
    for index, plant in enumerate(plants):
        cv2.imshow(f"Plant {index+1}", plant)
    if prev_plant_count > len(plants):
        [cv2.destroyWindow(f"Plant {plant_index+1}") for plant_index in range(len(plants), prev_plant_count)]
    prev_plant_count = len(plants)
    #cv2.imwrite(f"{folder_name}/{image_count}.jpg", final_image)
    cv2.imshow("Camera", rectangle_final_image)
    cv2.imshow("Mask", mask)
    cv2.waitKey(1)
    image_count += 1
def getDepth(ros_depth):
    cv2.imshow("Depth", bridge.imgmsg_to_cv2(ros_depth))
    cv2.waitKey(1)
def getPosition(ros_position):
    global position
    position = ros_position
def minDistWithYaw(position_1, position_2):
    return math.sqrt((position_1[0]-position_2[0])**2+(position_1[1]-position_2[1])**2+(position_1[2]-position_2[2])**2+(position_1[3]-position_2[3])**2)
def filterPlantFrames(plant_positions):
    global plant_frames
    filtered_plant_frames = {}
    for plant_position in plant_positions:
        x, y, z, yaw = plant_position["x"], plant_position["y"], plant_position["z"], plant_position["yaw"]
        required, minDist = None, 10000000000
        for drone_position in plant_frames.keys():
            x_drone, y_drone, z_drone, yaw_drone = drone_position[0], drone_position[1], drone_position[2], drone_position[3]
            current_dist = minDistWithYaw([x, y, z, yaw], [x_drone, y_drone, z_drone, yaw_drone])
            if current_dist < minDist:
                required = drone_position
                minDist = current_dist
        filtered_plant_frames[tuple(plant_position.values())] = plant_frames[required]
    return filtered_plant_frames
def filterCenterPlant(plant_positions):
    global width
    filtered_plant_frames = {}
    for position, current_plant_frames in plant_positions.items():
        '''
        x_center_positions = {}
        for current_plant_frame_rectangle in current_plant_frames.keys():
            x_center_positions[abs(width-(current_plant_frame_rectangle[0][0]+current_plant_frame_rectangle[1][0])/2)] = current_plant_frame_rectangle
        '''
        filtered_plant_frames[position] = []
        for plant_frame in current_plant_frames.values():
            filtered_plant_frames[position].append(plant_frame)
        #if len(x_center_positions) == 0:
        #    continue
        #filtered_plant_frames[position] = current_plant_frames[x_center_positions[min(list(x_center_positions.keys()))]]
    return filtered_plant_frames

if __name__ == "__main__":
    cwd = Path.cwd()
    folder = cwd / folder_name
    folder.mkdir(exist_ok=True)
    rospy.init_node("icuas")
    rate = rospy.Rate(10)
    plant_topic_data = "Pepper 10 11 14 21 23 24"                                             # Add Data after subscribing
    plant_type = plant_topic_data.split(' ')[0]
    plant_locations = [int(shelf_number) for shelf_number in plant_topic_data.split(' ')[1:]]
    trajectory_subscriber = rospy.Subscriber("/in_trajectory", Bool, getTrajectoryStatus)
    image_subscriber = rospy.Subscriber("/red/camera/color/image_raw", Image, getImage)
    #depth_subscriber = rospy.Subscriber("/red/camera/depth/image_raw", Image, getDepth)
    position_subscriber = rospy.Subscriber("/red/pose", PoseStamped, getPosition)
    imu_subscriber = rospy.Subscriber("/red/imu", Imu, getImuData)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except Exception as e:
        print(e)
    cv2.destroyAllWindows()
    print("Filtering Dictionary")  
    plant_position_data = []
    for plant_position in plant_locations:
        plant_position_data.append({"type": plant_type, "x": plant_center_frame_position[plant_position][0], "y": plant_center_frame_position[plant_position][1], "z": plant_center_frame_position[plant_position][2], "yaw": 1.570700})
        plant_position_data.append({"type": plant_type, "x": plant_center_frame_position[plant_position][0], "y": plant_center_frame_position[plant_position][1], "z": plant_center_frame_position[plant_position][2], "yaw": -1.570700})
    #with open("plant_des.csv", 'r') as file:
    #    plant_position_data = [line.split(',')[1:] for line in file.read().split('\n')[1:] if line != '']
    #    plant_position_data = [{"type": plant_data[1], "x": float(plant_data[2]), "y": float(plant_data[3]), "z": float(plant_data[4]), "yaw": 1.570700} for plant_data in plant_position_data]
    #    plant_position_data.extend([{"type": plant_data["type"], "x": float(plant_data["x"]), "y": float(plant_data["y"]), "z": float(plant_data["z"]), "yaw": -1.570700} for plant_data in plant_position_data])
    filtered_dictionary = filterPlantFrames(plant_position_data)
    print(filtered_dictionary)
    filtered_center_dictionary = filterCenterPlant(filtered_dictionary)
    print(filtered_center_dictionary)
    print("Filtered Dictionary")
    print(f"Writing Frames to folder {folder_name}")
    for position, plant_frames in filtered_center_dictionary.items():
        for index, plant_frame in enumerate(plant_frames):
            cv2.imwrite(f"{folder_name}/{position[0]}_{position[1]}_{position[2]}_{position[3]}_{position[4]}_{index}.jpg", all_plant_frames[plant_frame])
            print(f"{position[0]}_{position[1]}_{position[2]}_{position[3]}_{position[4]}_{index}.jpg")
    print(f"Done Writing Frames in format type_x_y_z_yaw.jpg")
    '''
    publishers = {}
    for index, (position, plant_frame) in enumerate(filtered_center_dictionary.items()):
        if position[4] == 1.5707:
            publishers[rospy.Publisher(f"/plant/frame/front/{index+1}", Image, queue_size=10)] = plant_frame
        else:
            publishers[rospy.Publisher(f"/plant/frame/back/{index-39}", Image, queue_size=10)] = plant_frame
    while True:
        for publisher, plant_frame in publishers.items():
            publisher.publish(bridge.cv2_to_imgmsg(all_plant_frames[plant_frame]))
    '''