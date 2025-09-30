import json
import cv2
import numpy as np
from pathlib import Path

def load_annotation_data(json_path):
    """JSON 파일에서 어노테이션 데이터 로드"""
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data

def draw_segmentation_with_overlay(image_path, annotation_data, output_path=None, opacity=0.4):
    """
    세그멘테이션 폴리곤을 반투명 오버레이로 그리기
    
    Args:
        image_path: 원본 이미지 경로
        annotation_data: JSON 어노테이션 데이터
        output_path: 저장할 이미지 경로 (None이면 표시만)
        opacity: 오버레이 투명도 (0.0 ~ 1.0, 낮을수록 투명)
    """
    
    # 이미지 로드
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Cannot load image from {image_path}")
        return
    
    # 오버레이용 이미지 복사본 생성
    overlay = img.copy()
    
    # 라벨별 색상 정의 (BGR 형식)
    label_colors = {
        'person': (255, 0, 128),    # 핑크 (사람과 비슷한 색)
        'car': (128, 0, 255),        # 보라색 (로봇카)
        'road': (0, 255, 255),       # 노란색 (도로)
        'default': (0, 255, 0)       # 초록색 (기본)
    }
    
    # 그리기 순서 정의 (road를 먼저 그리고, 그 위에 car와 person을 그림)
    draw_order = ['road', 'person', 'car']
    
    # 박스를 라벨별로 정렬
    sorted_boxes = []
    for order_label in draw_order:
        for box in annotation_data['boxes']:
            if box['label'] == order_label:
                sorted_boxes.append(box)
    
    # draw_order에 없는 라벨들 추가
    for box in annotation_data['boxes']:
        if box['label'] not in draw_order:
            sorted_boxes.append(box)
    
    # 각 박스(폴리곤) 그리기
    for box in sorted_boxes:
        label = box['label']
        color = label_colors.get(label, label_colors['default'])
        
        # 폴리곤 포인트 배열 생성
        points = np.array(box['points'], dtype=np.int32)
        
        # 채워진 폴리곤 그리기 (오버레이에)
        cv2.fillPoly(overlay, [points], color)
        
        # 폴리곤 외곽선 그리기 (원본 이미지에 직접)
        cv2.polylines(img, [points], True, color, 3)
        
        # 바운딩 박스 좌표 계산
        min_x = np.min(points[:, 0])
        min_y = np.min(points[:, 1])
        max_x = np.max(points[:, 0])
        max_y = np.max(points[:, 1])
        
        # 바운딩 박스 그리기 (원본 이미지에)
        cv2.rectangle(img, (min_x, min_y), (max_x, max_y), color, 3)
    
    # 반투명 오버레이 적용
    result = cv2.addWeighted(overlay, opacity, img, 1 - opacity, 0)
    
    # 라벨 텍스트 추가 (바운딩 박스 내부 좌측 상단)
    for box in sorted_boxes:
        label = box['label']
        color = label_colors.get(label, label_colors['default'])
        box_id = box.get('id', '0')
        
        # 폴리곤 포인트 배열 생성
        points = np.array(box['points'], dtype=np.int32)
        
        # 바운딩 박스 좌표
        min_x = np.min(points[:, 0])
        min_y = np.min(points[:, 1])
        
        # 0.8~1.0 사이의 랜덤 신뢰도 생성
        confidence = np.random.uniform(0.8, 1.0)
        
        # 텍스트 설정 (라벨 (id) 신뢰도)
        text = f"{label} ({box_id}) {confidence:.2f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.9
        thickness = 2
        
        # 텍스트 위치 (바운딩 박스 내부 좌측 상단)
        text_x = min_x + 10
        text_y = min_y + 30
        
        # 텍스트 그리기 (라벨과 같은 색상, 배경 없음)
        cv2.putText(result, text, 
                   (text_x, text_y),
                   font, font_scale, color, thickness)
    
    # 결과 저장 또는 표시
    if output_path:
        cv2.imwrite(output_path, result)
        print(f"Image saved to {output_path}")
    
    # 화면에 표시 (옵션)
    cv2.imshow('Segmentation with Transparent Overlay', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return result

def main():
    # 파일 경로 설정
    json_path = 'annotation.json'  # JSON 파일 경로
    image_path = '2.jpg'  # 이미지 파일 경로
    output_path = 'segmentation_result.jpg'  # 출력 파일 경로
    
    # JSON 데이터 로드
    annotation_data = load_annotation_data(json_path)
    
    # 세그멘테이션 시각화 (투명도 40%)
    result = draw_segmentation_with_overlay(
        image_path=image_path,
        annotation_data=annotation_data,
        output_path=output_path,
        opacity=0.4  # 투명도 조절 (0.0 ~ 1.0)
    )
    
    # 각 라벨별 객체 수 출력
    label_counts = {}
    for box in annotation_data['boxes']:
        label = box['label']
        label_counts[label] = label_counts.get(label, 0) + 1
    
    print("\n=== Detection Results ===")
    for label, count in label_counts.items():
        print(f"{label}: {count} object(s)")
    print(f"Total objects: {len(annotation_data['boxes'])}")
    print(f"Image size: {annotation_data['width']} x {annotation_data['height']}")

if __name__ == "__main__":
    main()