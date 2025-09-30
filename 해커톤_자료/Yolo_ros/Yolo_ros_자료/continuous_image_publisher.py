#!/usr/bin/env python3
"""
연속 이미지 퍼블리셔 노드
하나의 이미지를 카메라처럼 계속 발행
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class ContinuousImagePublisher(Node):
    def __init__(self):
        super().__init__('continuous_image_publisher')
        
        # 파라미터 선언
        self.declare_parameter('image_path', 'test_image.jpg')
        self.declare_parameter('fps', 30.0)  # 카메라처럼 30fps
        
        # 파라미터 가져오기
        image_path = self.get_parameter('image_path').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 이미지 퍼블리셔 생성
        self.image_publisher = self.create_publisher(Image, 'camera/camera/color/image_raw', 10)
        
        # 이미지 파일 로드 (한 번만 로드)
        self.image_path = self._find_image_file(image_path)
        self.image = cv2.imread(self.image_path)
        
        if self.image is None:
            self.get_logger().error(f'이미지를 로드할 수 없습니다: {self.image_path}')
            return
        
        # 이미지 정보 출력
        height, width, channels = self.image.shape
        self.get_logger().info(f'이미지 정보:')
        self.get_logger().info(f'  - 파일: {os.path.basename(self.image_path)}')
        self.get_logger().info(f'  - 해상도: {width}x{height}')
        self.get_logger().info(f'  - 채널: {channels}')
        self.get_logger().info(f'  - 발행 FPS: {fps}')
        
        # 타이머 생성 (지정된 FPS로 계속 발행)
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.publish_image)
        
        self.frame_count = 0
        self.get_logger().info('연속 이미지 퍼블리셔가 시작되었습니다.')
    
    def _find_image_file(self, image_path):
        """이미지 파일 경로 찾기"""
        if os.path.isabs(image_path):
            return image_path
        
        possible_paths = [
            os.path.join(os.getcwd(), image_path),
            os.path.join(os.path.expanduser('~'), 'ros2_ws', image_path),
            os.path.join('/home/nvidia/Desktop/AI_test', image_path),
            image_path
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                return os.path.abspath(path)
        
        return image_path
    
    def publish_image(self):
        """이미지를 ROS2 이미지 메시지로 발행 (매번 동일한 이미지)"""
        try:
            # OpenCV 이미지를 ROS2 이미지 메시지로 변환
            image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프는 매번 새로 생성
            image_msg.header.frame_id = 'camera_frame'
            
            # 이미지 발행
            self.image_publisher.publish(image_msg)
            
            self.frame_count += 1
            
            # 30프레임마다 로그 출력
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'프레임 {self.frame_count} 발행 완료')
                        
        except Exception as e:
            self.get_logger().error(f'이미지 발행 중 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        publisher = ContinuousImagePublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print('키보드 인터럽트로 종료됩니다.')
    except Exception as e:
        print(f'오류 발생: {e}')
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()