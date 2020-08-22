# Opencv_Car_Speed_Detection_System

This is a simple program which detects and tracks speed of car going on the road using opencv and python 

## Installation


* Step1: Install Python3 with PIP
```shell
Follow this Video procedure: https://www.youtube.com/watch?v=mFqdeX1C-8M
```
* Step2: Install VS and Support Binaries (Optional)
```shell
Follow this Video procedure: https://www.youtube.com/watch?v=DIw02CaEusY
```
* Step3: Install the Required Libraries, Open Command Prompt and Type
```bash
pip3 install -r requirements.txt 
```

## Usage

Open Command Prompt and Type:
```bash
python3 speed_estimation_dl_video.py
```
## SideNote:
````shell
if you want to use Webcam please change the following line in speed_estimation_dl_video.py 
Line[15] to : cap = cv2.VideoCapture(0)

if you want to use Video File please change the following line in speed_estimation_dl_video.py 
Line[15] to : cap = cv2.VideoCapture(<Video_Path>)
 
````

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.


## License
[MIT](https://choosealicense.com/licenses/mit/)
