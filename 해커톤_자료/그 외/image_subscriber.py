#!/usr/bin/env python3
"""
이미지 토픽 구독자 노드
test_video_publisher에서 발행하는 이미지를 구독하여 정보 출력
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 이미지 구독자 생성
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.frame_count = 0
        self.get_logger().info('이미지 구독자가 시작되었습니다. /camera/image_raw 토픽을 구독합니다.')
        
    def image_callback(self, msg):
        """이미지 메시지 콜백 함수"""
        try:
            # ROS2 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            self.frame_count += 1
            
            # 10프레임마다 정보 출력
            if self.frame_count % 10 == 0:
                height, width, channels = cv_image.shape
                self.get_logger().info(
                    f'프레임 {self.frame_count}: {width}x{height}, '
                    f'채널: {channels}, 타임스탬프: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
                )
                
            # 선택사항: 이미지를 파일로 저장 (처음 몇 프레임만)
            if self.frame_count <= 3:
                filename = f'received_frame_{self.frame_count:03d}.jpg'
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'프레임 저장: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        image_subscriber = ImageSubscriber()
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        print('키보드 인터럽트로 종료됩니다.')
    finally:
        if 'image_subscriber' in locals():
            image_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
