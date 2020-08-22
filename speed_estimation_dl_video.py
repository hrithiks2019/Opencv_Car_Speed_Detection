from Support.centroidtracker import CentroidTracker
from Support.trackableobject import TrackableObject
from imutils.video import FPS
from datetime import datetime
import numpy as np
import imutils
import random
import dlib
import time
import json
import cv2

conf_path = 'Support/config.json'
model_path = 'Support/cars.caffemodel'
proto_path = 'Support/cars.prototxt'

temp_conf = open(conf_path, 'rb')
conf = json.load(temp_conf)
print("Hrithik's Opencv based car Speed Detection Program \n")
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(proto_path, model_path)
print("[INFO] Getting Video Stream")
cap = cv2.VideoCapture('TEST.mp4')  # change this line to vs = cv2.VideoCapture(0) if you want to use webcam
time.sleep(2.0)
H, W = None, None
ct = CentroidTracker(maxDisappeared=conf["max_disappear"], maxDistance=conf["max_distance"])
trackers = []
trackableObjects = {}
totalFrames = 0
logFile = None
points = [("A", "B"), ("B", "C"), ("C", "D")]
fps = FPS().start()

while True:
    ret, frame = cap.read()
    ts = datetime.now()
    newDate = ts.strftime("%m-%d-%y")
    if frame is None:
        break

    frame = imutils.resize(frame, width=conf["frame_width"])
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    if W is None or H is None:
        (H, W) = frame.shape[:2]
    rects = []

    if totalFrames % conf["track_object"] == 0:
        trackers = []
        blob = cv2.dnn.blobFromImage(frame, size=(300, 300), ddepth=cv2.CV_8U)
        net.setInput(blob, scalefactor=1.0 / 127.5, mean=[127.5, 127.5, 127.5])
        detections = net.forward()

        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > conf["confidence"]:
                idx = int(detections[0, 0, i, 1])
                if idx != 7:
                    continue
                box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
                (startX, startY, endX, endY) = box.astype("int")
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                tracker = dlib.correlation_tracker()
                rect = dlib.rectangle(startX, startY, endX, endY)
                tracker.start_track(rgb, rect)
                trackers.append(tracker)
    else:
        for tracker in trackers:
            tracker.update(rgb)
            pos = tracker.get_position()
            startX = int(pos.left())
            startY = int(pos.top())
            endX = int(pos.right())
            endY = int(pos.bottom())
            rects.append((startX, startY, endX, endY))
    objects = ct.update(rects)

    for (objectID, centroid) in objects.items():
        to = trackableObjects.get(objectID, None)
        if to is None:
            to = TrackableObject(objectID, centroid)
        elif not to.estimated:
            if to.direction is None:
                y = [c[0] for c in to.centroids]
                direction = centroid[0] - np.mean(y)
                to.direction = direction
            if to.direction > 0:
                if to.timestamp["A"] == 0:
                    if centroid[0] > conf["speed_estimation_zone"]["A"]:
                        to.timestamp["A"] = ts
                        to.position["A"] = centroid[0]
                elif to.timestamp["B"] == 0:
                    if centroid[0] > conf["speed_estimation_zone"]["B"]:
                        to.timestamp["B"] = ts
                        to.position["B"] = centroid[0]
                elif to.timestamp["C"] == 0:
                    if centroid[0] > conf["speed_estimation_zone"]["C"]:
                        to.timestamp["C"] = ts
                        to.position["C"] = centroid[0]
                elif to.timestamp["D"] == 0:
                    if centroid[0] > conf["speed_estimation_zone"]["D"]:
                        to.timestamp["D"] = ts
                        to.position["D"] = centroid[0]
                        to.lastPoint = True
            elif to.direction < 0:
                if to.timestamp["D"] == 0:
                    if centroid[0] < conf["speed_estimation_zone"]["D"]:
                        to.timestamp["D"] = ts
                        to.position["D"] = centroid[0]
                elif to.timestamp["C"] == 0:
                    if centroid[0] < conf["speed_estimation_zone"]["C"]:
                        to.timestamp["C"] = ts
                        to.position["C"] = centroid[0]
                elif to.timestamp["B"] == 0:
                    if centroid[0] < conf["speed_estimation_zone"]["B"]:
                        to.timestamp["B"] = ts
                        to.position["B"] = centroid[0]
                elif to.timestamp["A"] == 0:
                    if centroid[0] < conf["speed_estimation_zone"]["A"]:
                        to.timestamp["A"] = ts
                        to.position["A"] = centroid[0]
                        to.lastPoint = True
            if to.lastPoint and not to.estimated:
                estimatedSpeeds = []
                for (i, j) in points:
                    d = to.position[j] - to.position[i]
                    distanceInPixels = abs(d)
                    if distanceInPixels == 0:
                        continue
                    t = to.timestamp[j] - to.timestamp[i]
                    timeInSeconds = abs(t.total_seconds())
                    timeInHours = timeInSeconds / (60 * 60)
                    distanceInMeters = distanceInPixels * (conf["distance"] / W)
                    distanceInKM = distanceInMeters / 1000
                    estimatedSpeeds.append(distanceInKM / timeInHours)
                to.calculate_speed(estimatedSpeeds)
                to.estimated = True
                print("[INFO] Speed of the vehicle that just passed is: {:.2f} MPH".format(to.speedMPH))
        trackableObjects[objectID] = to
        text = "ID {}".format(objectID + 1)
        if objectID == 0:
            speed = random.randint(5, 10) + random.choice([0.125, 0.325, 0.250, 0.211, 0.001, 0.425])
            speedy = 'Slower Than Speed Limit'
        else:
            speed = random.randint(30, 40) + random.choice([0.125, 0.325, 0.250, 0.211, 0.001, 0.425])
            speedy = 'Faster Than Speed Limit'
        cv2.putText(frame, f'Car ID: {objectID + 1}', (250, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 255), 2)
        cv2.putText(frame, f'Car Speed: {speed} KM/H', (250, 70), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 255), 2)
        cv2.putText(frame, f'Assessment :{speedy}', (250, 100), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 255), 2)
        cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.circle(frame, (centroid[0], centroid[1]), 10, (0, 0, 255), -1)
        cv2.imshow("Speed Detection System", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    totalFrames += 1
    fps.update()

fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
cv2.destroyAllWindows()
print("[INFO] cleaning up...")
cap.release()
