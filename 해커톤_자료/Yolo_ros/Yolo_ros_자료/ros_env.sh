#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────
# 공통 ROS 2 환경설정 (모든 터미널, 모든 머신에서 동일하게 source)
# ─────────────────────────────────────────────────────────────

# 1) (선택) 가상환경 비활성화 — conda/venv에서 PATH 충돌 방지
conda deactivate 2>/dev/null || true

# 2) ROS 2 Humble 환경 로드
source /opt/ros/humble/setup.bash

# 3) 멀티머신 통신에 필요한 공통 변수
export ROS_DOMAIN_ID=13                  # 같은 도메인 번호여야 서로 보임
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # 또는 rmw_cyclonedds_cpp (양쪽 동일!)
export FASTDDS_SHM_TRANSPORT_DISABLE=1   # FastDDS일 때 SHM 비활성 — 멀티머신 안정성↑
export ROS_LOCALHOST_ONLY=0              # 멀티머신이면 반드시 0 (1이면 같은 PC에서만 보임)

# 4) (선택) 이미지 트랜스포트 기본값
# 대부분 YOLO는 raw를 구독하지만, 다른 노드는 compressed를 원할 수 있어 참고용
# export IMAGE_TRANSPORT=compressed

# 5) ROS 2 데몬 리스타트 (그래프 갱신 문제 예방)
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start

# 6) 현재 설정 프린트 (디버깅용)
echo "[ros_env] DOMAIN=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION LOCAL=$ROS_LOCALHOST_ONLY"
