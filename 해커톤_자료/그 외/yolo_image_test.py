#!/usr/bin/env python3
"""
YOLO 모델 이미지 테스트 스크립트
ROS 없이 단일 이미지로 학습된 모델 성능 테스트
"""

import cv2
import sys
from ultralytics import YOLO
import numpy as np

def test_yolo_model(model_path, image_path, threshold=0.25):
    """
    YOLO 모델로 단일 이미지 테스트
    """
    print(f"모델 로딩: {model_path}")
    model = YOLO(model_path)
    
    print(f"이미지 로딩: {image_path}")
    image = cv2.imread(image_path)
    if image is None:
        print(f"이미지를 읽을 수 없습니다: {image_path}")
        return
    
    print(f"추론 실행... (threshold: {threshold})")
    results = model.predict(
        source=image,
        verbose=True,
        conf=threshold,
        iou=0.5,
        imgsz=640,
        device='cuda:0',
        save=True,  # 결과 이미지 저장
        save_txt=True  # 텍스트 결과 저장
    )
    
    result = results[0]
    
    # 탐지 결과 분석
    print("\n=== 탐지 결과 ===")
    if result.boxes is not None and len(result.boxes) > 0:
        print(f"총 {len(result.boxes)}개 객체 탐지됨")
        
        for i, box in enumerate(result.boxes):
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = model.names[class_id]
            
            print(f"객체 {i+1}: {class_name} (ID: {class_id}, 신뢰도: {confidence:.3f})")
        
        # 특별히 person, car, road, sidewalk 클래스 확인
        target_classes = ['person', 'car', 'road', 'sidewalk']
        for target in target_classes:
            count = 0
            for box in result.boxes:
                class_name = model.names[int(box.cls[0])]
                if class_name == target:
                    count += 1
            if count > 0:
                print(f"✓ {target}: {count}개 탐지됨")
            else:
                print(f"✗ {target}: 탐지되지 않음")
    else:
        print("탐지된 객체가 없습니다.")
    
    # 세그멘테이션 마스크 정보
    if result.masks is not None:
        print(f"\n세그멘테이션 마스크: {len(result.masks)}개")
    
    print(f"\n결과 이미지 저장됨: runs/detect/predict/")
    print("저장된 결과를 확인해보세요.")

def main():
    if len(sys.argv) < 3:
        print("사용법: python yolo_image_test.py <모델경로> <이미지경로> [threshold]")
        print("예시: python yolo_image_test.py best.pt street_image.jpg 0.25")
        sys.exit(1)
    
    model_path = sys.argv[1]
    image_path = sys.argv[2]
    threshold = float(sys.argv[3]) if len(sys.argv) > 3 else 0.25
    
    print("="*50)
    print("YOLO 이미지 테스트")
    print("="*50)
    
    test_yolo_model(model_path, image_path, threshold)

if __name__ == "__main__":
    main()
