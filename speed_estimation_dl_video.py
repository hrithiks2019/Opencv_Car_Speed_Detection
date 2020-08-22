import json
import numpy as np
import cv2
import imutils
import serial
import csv
import datetime

conf_path = 'Support/config.json'
model_path = 'Support/cars.caffemodel'
proto_path = 'Support/cars.prototxt'
temp_conf = open(conf_path, 'rb')
conf = json.load(temp_conf)
net = cv2.dnn.readNetFromCaffe(proto_path, model_path)
cap = cv2.VideoCapture('TEST.mp4')  # change this line to vs = cv2.VideoCapture(0) if you want to use webcam
W, H = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
time_per_frame = (1 / (cap.get(cv2.CAP_PROP_FPS)))
r_q, crossed, f_c = [(int(W * 0.25)), (int(W * 0.50)), (int(W * 0.75))], [], 0
final, tmp_fr_nm, id = [], [], 1
if conf['output_arduino']:
    ser = serial.Serial('COM4', 115200, timeout=1)


def calc_speed(time_taken):
    m = conf['quadrant_length'] / (time_taken * time_per_frame)
    km = m * 6
    mi = km * conf['MI_KM']
    if km < conf['speed_limit']:
        g_fl = 'OPEN'
    else:
        g_fl = 'CLOSE'
    return km, mi, m, g_fl


while True:
    now_time = datetime.datetime.now().strftime('%d/%m/%Y %H:%M:%S')
    ret, frame = cap.read()
    if ret:
        f_c += 1
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        blob = cv2.dnn.blobFromImage(frame, size=(300, 300), ddepth=cv2.CV_8U)
        net.setInput(blob, scalefactor=1.0 / 127.5, mean=[127.5, 127.5, 127.5])
        detections = net.forward()
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > conf['confidence']:
                idx = int(detections[0, 0, i, 1])
                if idx != 7:
                    continue
                box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
                (startX, startY, endX, endY) = box.astype("int")
                cv2.circle(frame, (int((startX + endX) / 2), int((startY + endY) / 2)), 10, (0, 0, 255), -1)
                c_cent = int((startX + endX) / 2)
                if (c_cent > r_q[0]) and (c_cent < r_q[1]) and (c_cent < r_q[2]):
                    if 's1' not in crossed:
                        crossed.append('s1')
                        tmp_fr_nm.append(f_c)
                elif (c_cent > r_q[0]) and (c_cent > r_q[1]) and (c_cent < r_q[2]):
                    if 's2' not in crossed:
                        crossed.append('s2')
                        tmp_fr_nm.append(f_c)
                elif (c_cent > r_q[0]) and (c_cent > r_q[1]) and (c_cent > r_q[2]) and (c_cent < 1500):
                    if 's3' not in crossed:
                        crossed.append('s3')
                        tmp_fr_nm.append(f_c)
                elif ('s3' in crossed) and (c_cent > 1500):
                    tempi = (((tmp_fr_nm[1] - tmp_fr_nm[0]) + (tmp_fr_nm[2] - tmp_fr_nm[1])) / 2)
                    speed_km, speed_mi, speed_m, gate_flag = calc_speed(tempi)
                    final.append([id, now_time, f'{speed_km} Km/hr', f'{speed_mi} Mi/hr', gate_flag])
                    print('\n Details of Past Car:')
                    print(f'Car Id {id}')
                    print(f'speed of the car .= {speed_m} M/s')
                    print(f'speed of the car in Metric.std = {speed_km} KM/hr')
                    print(f'speed of the car in Imperial.std= {speed_mi} M/hr')
                    if conf['output_arduino']:
                        if gate_flag == 'OPEN':
                            ser.write(bytes((str('O') + '\n'), 'ascii'))
                        else:
                            ser.write(bytes((str('C') + '\n'), 'ascii'))
                        print(f'Gate Control Status: {gate_flag}')
                    else:
                        print(f'Gate Control Status: {gate_flag}')
                    crossed, tmp_fr_nm, tempi, id = [], [], 0, (id + 1)
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(frame, f'ID: {id}', (startX, startY - 15), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 255, 255), 2, cv2.LINE_AA)

        frame = imutils.resize(frame, width=conf['width'])
        cv2.imshow("Speed Detection System", frame)
        cv2.waitKey(1)
    else:
        break

if conf['output_csv']:
    with open(conf['output_csv_file'], 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["SN", "Date", "Speed(KPH)", "Speed (MIH)", "Gate_status"])
        for i in final:
            writer.writerow(i)

cv2.destroyAllWindows()
cap.release()
