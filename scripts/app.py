import cv2
import argparse
import numpy as np
import math

argparser = argparse.ArgumentParser(description='Simple implementation of Yolov3 algorithm in Python, using custom Dataset.')

argparser.add_argument('--img', type=str)
argparser.add_argument('--out', type=str)

args = argparser.parse_args()

confidence, threshold = 0.5, 0.3

# Load the Custom class labels, Weights and Config files
# Then create the DNN model
labelPath = './obj.names'
labels = open(labelPath).read().strip().split('\n')
weightsPath = './yolov4-tiny-custom_best.weights'
configPath = './yolov4-tiny-custom.cfg'
net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

# Get all layer names
# Then get all [yolo] layers
layer_names = net.getLayerNames()
yolo_layers = ['yolo_30', 'yolo_37']

def draw_bb(img, boxes, confidences, classids, idxs, labels):
    # If detection exists
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

    return img

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
                print(detect_confidence)
                # scale the BB coordinates back relative to
                # the size of the image.
                # YOLO returns the center (x,y) - coordinates of BB
                # followed by the boxes weight and height
                box = detection[0:4] * np.array([width, height, width, height])
                centerX, centerY, box_width, box_height = box.astype('int')
                x = int(centerX - (box_width / 2))
                y = int(centerY - (box_height / 2))

                boxes.append([x, y, int(box_width), int(box_height), class_id])
                confidences.append(float(detect_confidence))
                class_ids.append(class_id)

    # Suppress overlapping boxes
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence, threshold)

    img = draw_bb(img, boxes, confidences, class_ids, idxs, labels)

    return img, boxes, confidences, class_ids, idxs

def run_detection(img, depth):
    print(img.shape)
    height, width = img.shape[:2]

    print("Running Detection!")
    img, boxes, _, _, _ = predict(net, yolo_layers, height, width, img, labels)
    x, y, z = get3DCoords(boxes, depth)

    return img


def get3DCoords(boxes, depth, fx, fy, ppx, ppy):

    for box in boxes:
        x, y = box[0], box[1]
        w, h = box[2], box[3]
        center_x = x + w // 2
        center_y = y + h // 2

        class_id = box[4]
        label = labels[class_id]

        dist = depth[center_x][center_y]

        Xtemp = dist*(center_x - ppx)/fx
        Ytemp = dist*(center_y - ppy)/fy
        Ztemp = dist
        theta = 0

        Xtarget = Xtemp - 0 #subtract RGB camera module offset from the center of the realsense
        Ytarget = -(Ztemp*math.sin(theta) + Ytemp*math.cos(theta))
        Ztarget = Ztemp*math.cos(theta) + Ytemp*math.sin(theta)

        print(f"Found {label} at ({Xtarget}, {Ytarget}, {Ztarget})!")

    return Xtarget, Ytarget, Ztarget



