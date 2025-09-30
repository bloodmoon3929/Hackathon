import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from yolo_msgs.msg import DetectionArray  # YOLO detections

# ▼ 추가: 토글 시 POST 전송을 위한 의존성
import requests
from concurrent.futures import ThreadPoolExecutor

class PersonFlag(Node):
    def __init__(self):
        super().__init__('person_flag')

        # ── parameters
        self.declare_parameter('input_topic', '/yolo/detections')
        self.declare_parameter('output_flag_topic', '/pedestrian')  # 요청하신 토픽명 그대로
        self.declare_parameter('output_count_topic', '/person_count')
        self.declare_parameter('target_class_ids', [0])            # COCO: person=0
        self.declare_parameter('target_labels', ['person', 'human'])
        self.declare_parameter('min_score', 0.20)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('timeout_ms', 500)                  # 이 시간 True 없으면 False

        # ▼ 추가: 토글 시 POST 전송 관련 파라미터
        self.declare_parameter('post_url', 'http://localhost:5000/api/req_ped')
        self.declare_parameter('post_timeout_s', 0.8)

        # read params
        p = lambda k: self.get_parameter(k).get_parameter_value()
        self.input_topic        = p('input_topic').string_value
        self.output_flag_topic  = p('output_flag_topic').string_value
        self.output_count_topic = p('output_count_topic').string_value
        self.target_ids         = set(p('target_class_ids').integer_array_value)
        self.target_labels      = set([s.lower() for s in p('target_labels').string_array_value])
        self.min_score          = float(p('min_score').double_value)
        rate_hz                 = float(p('publish_rate_hz').double_value)
        self.timeout_ns         = int(p('timeout_ms').integer_value) * 1_000_000  # ms→ns

        # ▼ 추가: POST 관련 런타임 설정
        self.post_url           = p('post_url').string_value
        self.post_timeout_s     = float(p('post_timeout_s').double_value)

        # pub/sub
        self.pub_flag  = self.create_publisher(Bool, self.output_flag_topic, 10)
        self.pub_count = self.create_publisher(Int32, self.output_count_topic, 10)
        self.sub = self.create_subscription(DetectionArray, self.input_topic, self.cb, 10)

        # state
        self.last_true_ns = None
        self.last_count   = 0

        # ▼ 추가: 토글 감지용 상태
        self.prev_alive   = None

        # ▼ 추가: 비동기 POST 전송용 스레드 풀
        self._post_pool = ThreadPoolExecutor(max_workers=2)

        # periodic publish
        self.create_timer(1.0 / max(rate_hz, 0.1), self.tick)

        self.get_logger().info(
            f"[person_flag] IN:{self.input_topic} → OUT:{self.output_flag_topic},{self.output_count_topic} | "
            f"ids={list(self.target_ids)} labels={list(self.target_labels)} "
            f"min_score={self.min_score} timeout_ms={self.timeout_ns/1e6:.0f}"
            f"latitue={2359324432} longitude={9393592399}"
        )

    def _is_person(self, det) -> bool:
        try:
            if hasattr(det, 'score') and det.score < self.min_score:
                return False
        except Exception:
            pass
        try:
            if int(det.class_id) in self.target_ids:
                return True
        except Exception:
            pass
        try:
            name = (getattr(det, 'class_name', '') or '').lower()
            if name in self.target_labels:
                return True
        except Exception:
            pass
        return False

    def cb(self, msg: DetectionArray):
        count = 0
        for det in msg.detections:
            if self._is_person(det):  ### 이 부분
                self.get_logger().info(
                    f"labels={list(self.target_labels)} "
                    f"latitue={2359324432} longitude={9393592399}"
                )
                count += 1
        self.last_count = count
        if count > 0:
            self.last_true_ns = self.get_clock().now().nanoseconds

    # ▼ 추가: 토글 시 비동기 POST 전송
    def _post_toggle_async(self, alive: bool):
        payload = {"data": bool(alive)}
        headers = {"Content-Type": "application/json"}
        try:
            r = requests.post(self.post_url, json=payload, headers=headers, timeout=self.post_timeout_s)
            if r.status_code >= 400:
                self.get_logger().warn(f"POST {self.post_url} failed: {r.status_code} {r.text[:120]}")
            else:
                self.get_logger().info(f"POST {self.post_url} ok: {payload}")
        except Exception as e:
            self.get_logger().warn(f"POST {self.post_url} error: {e}")

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        alive = (self.last_true_ns is not None) and (now_ns - self.last_true_ns <= self.timeout_ns)

        # ▼ 추가: 토글 감지(상태 변화 시마다 POST)
        if self.prev_alive is None or alive != self.prev_alive:
            # 비동기 전송 (노드 실행 루프 블로킹 방지)
            self._post_pool.submit(self._post_toggle_async, alive)
            self.prev_alive = alive

        self.pub_flag.publish(Bool(data=alive))
        self.pub_count.publish(Int32(data=self.last_count if alive else 0))

def main():
    rclpy.init()
    rclpy.spin(PersonFlag())
    rclpy.shutdown()

if __name__ == '__main__':
    main()