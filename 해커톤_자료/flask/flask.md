파이썬의 개발자 테스트용 서버, 실제 배포로 사용하기에는 무리가 있음

node.js가 아닌 python으로 진행한 이유는 ros의 모든 노드를 python으로 만들었기 때문에 같은 환경인 파이썬으로 구성함
## 환경 설정
### conda 환경 생성

```bash
conda create -n flask python=3.8 -y
```

```bash
conda activate flask
```

### flask 관련 프레임워크 설치
```bash
pip install flask requests
pip install sqlalchemy
```

### 번외(curl 명령어를 통한 get, post)
```bash
curl -X POST http://localhost:5000/count -H "Content-Type: application/json" -d '{"value": 0}'
curl http://localhost:5000/count
```


## 📝 개별 탐지 로그 API

### POST /detection - 탐지 이벤트 저장
목적: 개별 객체 탐지 이벤트를 데이터베이스에 저장합니다.

요청 방법:
```bash
curl -X POST http://localhost:5000/detection \
  -H "Content-Type: application/json" \
  -d '{
    "class_label": 0,
    "object_id": 123
  }'
```


요청 데이터:
```json
json{
  "class_name": 0,    // 클래스 번호 (필수) - person:0, car:2 등
  "object_id": 123   // 객체 추적 ID (필수)
}
```

성공 응답:
```json
json{
  "status": "success",
  "message": "탐지 로그가 저장되었습니다",
  "detection": {
    "id": 1,
    "class_name": 0,
    "object_id": 123,
    "timestamp": "2024-08-29T14:30:00.123456"
  },
  "current_counters": [3, 0, 0, 0, 0]
}
```

참고: class_name이 0~4 범위면 실시간 카운터도 자동으로 업데이트됩니다.

### GET /detections - 탐지 로그 조회
목적: 저장된 탐지 로그를 필터링하여 조회합니다.
기본 조회:
```bash
curl http://localhost:5000/detections
```

필터링 옵션:
```bash
# 특정 클래스만 조회
curl "http://localhost:5000/detections?class_name=0"
```

## 특정 객체만 조회
```bash
curl "http://localhost:5000/detections?object_id=123"
```

## 특정 날짜 조회
```bash
curl "http://localhost:5000/detections?date=2024-08-29"
```

## 개수 제한
```bash
curl "http://localhost:5000/detections?limit=50"
```

## 복합 필터
```bash
curl "http://localhost:5000/detections?class_name=0&date=2024-08-29&limit=20"
```


응답:
```json
json{
  "status": "success",
  "detections": [
    {
      "id": 1,
      "class_name": 0,
      "object_id": 123,
      "frequency": 3,
      "timestamp": "2024-08-29T14:30:00.123456"
    }
  ],
  "total_count": 1,
  "filters": {
    "limit": 100,
    "class_name": 0,
    "object_id": null,
    "date": "2024-08-29"
  }
  
}
```
