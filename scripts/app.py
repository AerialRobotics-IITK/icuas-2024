import cv2
import argparse
import math
import numpy as np

argparser = argparse.ArgumentParser(description='Simple implementation of Yolov3 algorithm in Python, using custom Dataset.')

argparser.add_argument('--img', type=str)
argparser.add_argument('--out', type=str)

args = argparser.parse_args()

confidence, threshold = 0.5, 0.3

# Load the Custom class labels, Weights and Config files
# Then create the DNN model
labelPath = './custom_config/obj.names'
labels = open(labelPath).read().strip().split('\n')
weightsPath = './custom_config/yolov4-tiny-custom_best.weights'
configPath = './custom_config/yolov4-tiny-custom.cfg'
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

# Get all layer names
# Then get all [yolo] layers
layer_names = net.getLayerNames()
yolo_layers = ['yolo_30', 'yolo_37']


def draw_bb(img, boxes, confidences, classids, idxs, labels):
    # If detection exists
    boxes_with_labels = []
    if len(idxs):
        for i in idxs.flatten():
            # Get BB coordinates
            x, y = boxes[i][0], boxes[i][1]
            w, h = boxes[i][2], boxes[i][3]
            center_x = x + w // 2
            center_y = y + h // 2

            cv2.rectangle(img, (x,y), (x+w, y+h), (0, 255, 0), 3)
            cv2.circle(img, (center_x, center_y), 5, (255, 0, 0), -1)
            center_text = "({}, {})".format(center_x, center_y)
            cv2.putText(img, center_text, (center_x - 50, center_y - 10), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 1)

            text = "{}:{:2.5f}".format(labels[classids[i]], confidences[i])
            cv2.putText(img, text, (x, y-10), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 70, 0), 1)
            # print(f"Detected fruit at ({center_x},{center_y}) in image frame of shape {img.shape}")
            boxes_with_labels.append([x, y, w, h, labels[classids[i]]])
            # print(f"Sent box info with labels!")
    return img, boxes_with_labels

# init lists of detected boxes, confidences, class IDs
boxes, confidences, class_ids = [], [], []  

def predict(net, layer_names, height, width, img, labels):

    # Construct a blob from input
    # Then perform a forward pass of the yolo OD
    # Then get BB with associated probabilities
    blob = cv2.dnn.blobFromImage(img, 1/255.0, (640, 480), swapRB=True, crop=False)
    net.setInput(blob)
    layerOutputs = net.forward(layer_names)

    # init lists of detected boxes, confidences, class IDs
    boxes, confidences, class_ids = [], [], []
    # loop over each of the layer outputs
    for out in layerOutputs:
        # loop over each of the detections
        for detection in out:
            # extract the class ID and confidence of the current OD
            scores = detection[5:]
            class_id = np.argmax(scores)
            detect_confidence = scores[class_id]

            # filter out a weal predictions by ensuring the detected
            # probability is greater than minimum probability

            if detect_confidence > confidence:
                # scale the BB coordinates back relative to
                # the size of the image.
                # YOLO returns the center (x,y) - coordinates of BB
                # followed by the boxes weight and height
                box = detection[0:4] * np.array([width, height, width, height])
                centerX, centerY, box_width, box_height = box.astype('int')
                x = int(centerX - (box_width / 2))
                y = int(centerY - (box_height / 2))

                boxes.append([x, y, int(box_width), int(box_height)])

                confidences.append(float(detect_confidence))
                class_ids.append(class_id)

    # Suppress overlapping boxes
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence, threshold)

    img, boxes_with_labels = draw_bb(img, boxes, confidences, class_ids, idxs, labels)

    return img, boxes, confidences, class_ids, idxs, boxes_with_labels

def getDepthEstimate(roi_depth):
    n = 0
    sum = 0
    for i in range(0,roi_depth.shape[0]):
        for j in range(0,roi_depth.shape[1]):
            value = roi_depth.item(i, j)
            if value > 0.:
                n = n + 1
                sum = sum + value
    return sum / n # mean depth in the box
    

def coordinate_transformation(x, y, z):             # drone to gazebo format (x,y,z) to (-x,z,y)
    transformation_matrix = np.array([[0, 0, 1],
                                      [-1, 0, 0],
                                      [0, 1, 0]])
    original_coordinates = np.array([x, y, z])
    transformed_coordinates = np.dot(transformation_matrix, original_coordinates)  
    return transformed_coordinates

# def quaternionToRPY(quaternion):
#     x, y, z, w = quaternion
#     roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
#     pitch = np.arcsin(2 * (w * y - z * x))
#     yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
#     return roll, pitch, yaw



def get3DCoords(boxes, depth, fx = 381.36246688113556, fy = 381.36246688113556, ppx = 320.5, ppy = 240.5):

    fruits_pos = []

    for box in boxes:
        x, y = box[0], box[1]
        w, h = box[2], box[3]
        center_x = x + w // 2
        center_y = y + h // 2
        label = box[4]

        # roi_depth = depth[y:y+h, x:x+w] #selecting the box in depth image
        # dist = getDepthEstimate(roi_depth)

        # print(f"Trying to get depth of fruit at ({center_x},{center_y}) in depth frame of shape {depth.shape}")
        dist = depth[center_y][center_x]


        Xtemp = dist*(center_x - ppx)/fx
        Ytemp = dist*(center_y - ppy)/fy
        Ztemp = dist
        

        Xtarget, Ytarget, Ztarget =Xtemp,Ytemp,Ztemp

        print(f"Camera Frame pose for {label}: ({Xtarget}, {Ytarget}, {Ztarget})!")

        fruits_pos.append([Xtarget, Ytarget, Ztarget, label])

    return fruits_pos

def run_detection(img, depth):
    print(img.shape)
    height, width = img.shape[:2]
    img, boxes, _, _, _, boxes_with_labels = predict(net, yolo_layers, height, width, img, labels)

    fruits_pos = get3DCoords(boxes_with_labels, depth, fx = 381.36246688113556, fy = 381.36246688113556, ppx = 320.5, ppy = 240.5)

    return img, fruits_pos
