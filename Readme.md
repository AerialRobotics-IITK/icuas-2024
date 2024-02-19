**README**

## Dataset Conversion Script: convert.py

The `convert.py` script facilitates the conversion of datasets exported from Roboflow Darknet Zip into a format compatible with Darknet for training purposes.

### Usage:

```
python convert.py folder_name
```

Replace `folder_name` with the name of the folder containing the dataset exported from Roboflow.

---

## Prediction Application: app.py

The `app.py` script is designed for making predictions on images using YOLO (You Only Look Once) object detection model.

### Usage:

To save the prediction result as an image:
```
python app.py --img=test.jpg --out=predict.jpg
```

To display the prediction result using OpenCV (cv view):
```
python app.py --img=test.jpg
```

Replace `test.jpg` with the path to the image you want to make predictions on.

---


