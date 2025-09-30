# 깃허브 링크
https://github.com/mgonzs13/yolo_ros

Yolo의 불필요한 연산을 줄이기 위한 lifecycle을 사용하는 ros 기반의 yolo노드

# ROS 관점
ros_env.sh를 통해 ros2의 dds를 통해 환경을 연결

# 실행 방법
source ~/hack/start_camera.sh를 통해 카메라 실행
yolo.sh를 통해 바운딩박스의 yolo모델을 실행시킬 수 있음
2.sh를 통해 세그먼테이션의 yolo모델을 실행시킬 수 있음
person_flag_node.py를 통해 사람을 감지시 신호를 발생시킬수 있음
test_video_publisher_node를 통해 동영상 파일을 읽어서 프레임 단위로 ROS2 이미지 토픽으로 발행함


## AI 서버 실행 순서
1. Ter #1 (YOLO 실행)
    source ~/hack/yolo.sh
2. Ter #2 (rqt 모니터링)
    source ~/hack/ros_env.sh
    rqt
3. Ter #3 (/pedestrian 토픽 발행)
    source ~/workspace/install/setup.bash
    python ~/workspace/src/person_flag/person_flag/person_flag_node.py
4. Ter #4 (rc카와 tcp연결)
    (대회장 가서 gedit ~/rosTCP/ros2_topic_bool_client.py 하고 코드에서 TX2_IP = “IP값” 수정하기) python3 ~/rosTCP/ros2_topic_bool_client.py
5. Ter #5 (/pedestrian 토픽 모니터링)
    ros2 topic echo /pedestrian

## JETSON ORIGIN NANO 실행 순서
1. Ter #1 (카메라 켜기)
    source ~/hack/start_camera.sh
2. Ter #2 (카메라 화면 모니터링)
    source ~/hack/ros_env.sh
    rqt

## RC카 실행 순서
1. Ter #1
    cd wecar
    ros launch rececar teleop.xml
2. Ter #2
    cd tcp_server
    python tcp_bool_server.py
3. Ter #3
    rqt
    ex) 작동안할 시
    source /opt/ros/kinetic/setup.bash

## 테스트 영상 발행
1. 테스트 영상 발행
	source install/setup.bash
	ros2 run test_video_publisher test_video_publisher_node
2. 테스트 이미지 발행하기
	python3 continuous_image_publisher.py --ros-args -p image_path:={사용자가 원하는 이미지의 절대주소}
