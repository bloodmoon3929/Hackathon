#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────
# Desktop에서 YOLO 추론 노드 실행 (GPU 사용)
# ─────────────────────────────────────────────────────────────
set -e
source ~/hack/ros_env.sh
source ~/workspace/ros_ur_driver/install/setup.bash
# (중요) YOLO는 평시 compressed(image_raw/compressed)를 입력으로 받음 → compressed를 못받을 시, 위 republish의 /yolo/image_raw 사용
# 모델 경로는 절대경로 추천
#YOLO_MODEL="/home/nvidia/models/yolov8n.pt"
#YOLO_MODEL="/home/nvidia/Downloads/yolov8m-seg.pt"

# 성능 파라미터(런타임에 변경 가능하지만 초기에 적당 값 세팅)
#   half=true : FP16 (속도↑, 정확도 약간↓)
#   imgsz_*   : 모델 입력 해상도 (작을수록 빠름)
#   iou/max_det : NMS/최대 결과수
#   yolo_encoding : bgr8/rgb8 중 장면에 따라 맞는 쪽 선택






#ros2 launch yolo_bringup yolo.launch.py model:=best.pt\
ros2 launch yolo_bringup yolo.launch.py model:=yolov8m-seg.pt\
  input_image_topic:=/camera/camera/color/image_raw \
  target_frame:=camera_link \
  device:=cuda:0 \
  #model:=${YOLO_MODEL}\

