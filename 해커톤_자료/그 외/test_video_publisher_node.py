#!/usr/bin/env python3
"""
테스트 동영상 퍼블리셔 노드
동영상 파일을 읽어서 프레임 단위로 ROS2 이미지 토픽으로 발행
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from rclpy.parameter import Parameter


class TestVideoPublisher(Node):
    def __init__(self):
        super().__init__('test_video_publisher')
        
        # 파라미터 선언
        self.declare_parameter('video_path', '테스트 동영상.mp4')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('loop', True)
        
        # 파라미터 가져오기
        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        
        # 동영상 파일 경로 설정 (상대 경로 처리)
        if not os.path.isabs(video_path):
            # 여러 가능한 경로에서 파일 찾기
            possible_paths = [
                os.path.join(os.getcwd(), video_path),  # 현재 작업 디렉토리
                os.path.join(os.path.expanduser('~'), 'ros2_ws', video_path),  # 홈/ros2_ws
                os.path.join('/home/nvidia/ros2_ws/CITY', video_path),  # 수정된 절대 경로
                video_path  # 원본 경로
            ]
            
            full_video_path = None
            for path in possible_paths:
                if os.path.exists(path):
                    full_video_path = os.path.abspath(path)
                    break
            
            if full_video_path is None:
                full_video_path = video_path  # 기본값
        else:
            full_video_path = video_path
            
        self.get_logger().info(f'동영상 파일 경로: {full_video_path}')
        
        # OpenCV VideoCapture 초기화
        self.cap = cv2.VideoCapture(full_video_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'동영상 파일을 열 수 없습니다: {full_video_path}')
            return
            
        # 동영상 정보 출력
        total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info(f'동영상 정보:')
        self.get_logger().info(f'  - 총 프레임 수: {total_frames}')
        self.get_logger().info(f'  - 원본 FPS: {video_fps}')
        self.get_logger().info(f'  - 해상도: {width}x{height}')
        self.get_logger().info(f'  - 발행 FPS: {fps}')
        self.get_logger().info(f'  - 반복 재생: {self.loop}')
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 이미지 퍼블리셔 생성
        self.image_publisher = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        
        # 타이머 생성 (FPS에 따른 주기 계산)
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.frame_count = 0
        
        self.get_logger().info('테스트 동영상 퍼블리셔가 시작되었습니다.')
        
    def publish_frame(self):
        """프레임을 읽어서 ROS2 이미지 메시지로 발행"""
        ret, frame = self.cap.read()
        
        if not ret:
            if self.loop:
                # 동영상 다시 시작
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                self.get_logger().info('동영상을 다시 시작합니다.')
            else:
                self.get_logger().info('동영상 재생이 완료되었습니다.')
                self.cap.release()
                return
                
        if ret:
            try:
                # OpenCV 이미지를 ROS2 이미지 메시지로 변환
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'camera_frame'
                
                # 이미지 발행
                self.image_publisher.publish(image_msg)
                
                self.frame_count += 1
                if self.frame_count % 30 == 0:  # 30프레임마다 로그 출력
                    self.get_logger().info(f'프레임 {self.frame_count} 발행 완료')
                    
            except Exception as e:
                self.get_logger().error(f'프레임 발행 중 오류 발생: {e}')
                
    def destroy_node(self):
        """노드 종료 시 리소스 정리"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_video_publisher = TestVideoPublisher()
        rclpy.spin(test_video_publisher)
    except KeyboardInterrupt:
        print('키보드 인터럽트로 종료됩니다.')
    except Exception as e:
        print(f'오류 발생: {e}')
    finally:
        if 'test_video_publisher' in locals():
            test_video_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()