# ROS2를 활용한 Camera App Project
## 설치 방법
**1. Workspace 폴더 및 src 폴더 생성**
```
$ mkdir -p ~/<workspace name>/src
```
**2. Build**
``` 
# workspace 폴더에서 빌드
$ colcon build
```
**3. 패키지 생성**
```
# pkg nmae ros_camera가 아닐 경우 pkg name 수정필요 
$ ros2 pkg create --build-type ament_python ros_camera
```
**4. src 폴더에서 msgs pkg 생성**
```
$ cd src
$ ros2 pkg create --build-type ament_cmake ros_camera_msgs
```
**5. ros_camera 폴더 파일 경로 수정 (setup.py / package.xml)**

**6. ros_camera_msgs 폴더 파일 경로 수정**

**7. Github 파일 경로에 맞게 이동**

## 사용 방법
**1. 파일 경로 수정 후 빌드**
```
# workspace 폴더에서 빌드
$ colcon build
```
**2. 터미널 명령어 실행**
```
# ROS의 환경 변수 및 기타 설정을 로드
$ source /opt/ros/humble/setup.bash;

# ROS Domain 설정 
$ export ROS_DOMAIN_ID=8

# 빌드된 ROS 2 패키지의 실행 (경로는 사용자에 맞게 수정)
$ source ~/dev_ws/ROSCamera/install/local_setup.bash;
```
**3. launch file 실행**
```
# camera_app.launch.py 실행
# ros2 launch <pgg name> <launch file name>
$ ros2 launch ros_camera camera_app.launch.py
```
**4. 서비스 실행 방법**

**a. 이미지 저장**
```
# ros2 service call <service name> <service type>
$ ros2 service call /save_image ros_camera_msgs/srv/SaveImagem
```
**b. 비디오 저장**
```
# ros2 service call <service name> <service type> <request>
# duration: 녹화 시간(초)
$ ros2 service call /save_video ros_camera_msgs/srv/SaveVideo "{duration: 5}"
```
**c. filter 변경**
```
# ros2 service call <service name> <service type> <request>
# request 종류: canny, concave, camera, gray, cartoon
$ ros2 service call /change_topic ros_camera_msgs/srv/ChangeTopic "{topic: canny}"
```
**5. rqt사용**

- rqt GUI를 통해 cam화면 확인
![image](https://github.com/okotak99/ROS2_Camera/assets/157962186/ae545b87-9fb5-4a92-b344-1aa02a6257fb)


## 파라미터 설정
**camera_app.yaml 파라미터**
```
# img_publisher 파일의 송신 이미지 크기 
img_publisher:
  ros__parameters:
    width: 640
    length: 480

# camera_server파일의 image, video 저장 경로
camera_server:
  ros__parameters:
    image_path: "/home/addinedu/dev_ws/ROSCamera/src/ros_camera/resource/image/"
    video_path: "/home/addinedu/dev_ws/ROSCamera/src/ros_camera/resource/video/"
```

## 노드 설명
![image](https://github.com/okotak99/ROS2_Camera/assets/157962186/a26363e1-b986-489e-9515-64a6b6ee25ab)

**img_publisher**
- webcam을 통해 받은 frame을 /camera topic으로 publish
  
**filter1, filter2**
- /camera topic을 통해 받은 frame을 다양한 필터를 적용하여 publish
- canny, cartoon, gray, concave filter로 구성

**camera_server**
- service를 처리하는 서버 역할
- 이미지 저장 service
  - save image 서비스를 통해 현재 구독하고 있는 화면을 저장
- 비디오 저장 service
  - save video 서비스를 통해 현재 구독하고 있는 화면의 비디오를 저장
  - request: duration(초)을 입력해서 녹화시간 설정
- filter 변경
  - change topic 서비스를 통해 구독중인 화면을 변경
  - request: 각 발행하는 토픽의 이름을 입력해 필터 적용 (ex. canny)


