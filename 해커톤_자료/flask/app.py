from flask import Flask, request, jsonify
from flask_sqlalchemy import SQLAlchemy
from flask_cors import CORS
from datetime import datetime, timedelta
import threading
import time
import json

app = Flask(__name__)

CORS(app, resources={r"/api/*": {"origins": "*"}})  

# 데이터베이스 설정 (MySQL 사용)
# MySQL 연결 정보를 환경에 맞게 수정하세요
MYSQL_USER = 'root'  # MySQL 사용자명
MYSQL_PASSWORD = 'nvidia'  # MySQL 비밀번호
MYSQL_HOST = 'localhost'  # MySQL 서버 주소
MYSQL_PORT = 3306  # MySQL 포트
MYSQL_DATABASE = 'detection_db'  # 데이터베이스 이름

app.config['SQLALCHEMY_DATABASE_URI'] = f'mysql+pymysql://{MYSQL_USER}:{MYSQL_PASSWORD}@{MYSQL_HOST}:{MYSQL_PORT}/{MYSQL_DATABASE}'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

# 데이터베이스 모델 정의
class DetectionLog(db.Model):
    __tablename__ = 'detection_log'
    
    id = db.Column(db.Integer, primary_key=True, autoincrement=True)  # 자동 증가 고유 번호
    class_label = db.Column(db.Integer, nullable=False)  # 탐지된 객체의 클래스명에 대응되는 번호 (예: person:0, car:2)
    object_id = db.Column(db.String(50), nullable=False)  # 각 객체의 고유 식별자 (추적용 ID) 
    timestamp = db.Column(db.DateTime, default=datetime.now, nullable=False)  # 탐지 시각
    
    def to_dict(self):
        return {
            'id': self.id,
            'class_label': self.class_label,
            'object_id': self.object_id,
            'timestamp': self.timestamp.isoformat()
        }



# 전역 변수로 실시간 카운트 관리
last_reset_time = datetime.now()


# 스레드 안전성을 위한 락
counter_lock = threading.Lock()


user_status=False
pedestrian_last_updated = datetime.now()


@app.route('/detection', methods=['POST'])
def log_detection():
    """개별 탐지 이벤트를 데이터베이스에 저장"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({'error': 'JSON 데이터가 필요합니다'}), 400
        
        # 필수 필드 검증
        required_fields = ['class_label', 'object_id']
        for field in required_fields:
            if field not in data:
                return jsonify({'error': f'{field} 필드가 필요합니다'}), 400
        
        # 데이터 타입 검증
        if not isinstance(data['class_label'], int):
            return jsonify({'error': 'class_name은 정수여야 합니다 (예: person:0, car:2)'}), 400
        
        

        
        # 새 탐지 로그 생성
        new_detection = DetectionLog(
            class_label=data['class_label'],
            object_id=data['object_id']
        )
        
        db.session.add(new_detection)
        db.session.commit()
        
       
        
        return jsonify({
            'status': 'success',
            'message': '탐지 로그가 저장되었습니다',
            'detection': new_detection.to_dict()
        })
        
    except Exception as e:
        db.session.rollback()
        return jsonify({
            'error': f'탐지 로그 저장 중 오류: {str(e)}'
        }), 500

@app.route('/detections', methods=['GET'])
def get_detections():
    """저장된 탐지 로그 조회"""
    try:
        # 쿼리 파라미터
        limit = request.args.get('limit', 100, type=int)
        class_name = request.args.get('class_label', type=int)
        object_id = request.args.get('object_id', type=int)
        date_str = request.args.get('date')  # YYYY-MM-DD 형식
        
        query = DetectionLog.query
        
        # 필터링
        if class_label is not None:
            query = query.filter_by(class_label=class_label)
            
        if object_id is not None:
            query = query.filter_by(object_id=object_id)
            
        if date_str:
            try:
                target_date = datetime.strptime(date_str, '%Y-%m-%d').date()
                next_date = target_date + timedelta(days=1)
                query = query.filter(
                    DetectionLog.timestamp >= target_date,
                    DetectionLog.timestamp < next_date
                )
            except ValueError:
                return jsonify({'error': '날짜 형식이 올바르지 않습니다 (YYYY-MM-DD)'}), 400
        
        detections = query.order_by(DetectionLog.timestamp.desc()).limit(limit).all()
        
        return jsonify({
            'status': 'success',
            'detections': [detection.to_dict() for detection in detections],
            'total_count': len(detections),
            'filters': {
                'limit': limit,
                'class_label': class_label,
                'object_id': object_id,
                'date': date_str
            }
        })
        
    except Exception as e:
        return jsonify({
            'error': f'탐지 로그 조회 중 오류: {str(e)}'
        }), 500


@app.route('/api/req_ped', methods=['POST'])
def request_pedestrian():
    """보행자 상태 설정 API"""
    global user_status, pedestrian_last_updated
    
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({'error': 'JSON 데이터가 필요합니다'}), 400
        
        # 'data' 필드에서 boolean 값 추출
        if 'data' not in data:
            return jsonify({'error': 'data 필드가 필요합니다'}), 400
        
        pedestrian_data = data['data']
        
        # boolean 타입 검증
        if not isinstance(pedestrian_data, bool):
            return jsonify({'error': 'data 필드는 boolean 타입이어야 합니다 (true/false)'}), 400
        
        # 스레드 안전을 위한 락 사용하여 전역변수 업데이트
        with counter_lock:
            user_status = pedestrian_data
            pedestrian_last_updated = datetime.now()

        print(pedestrian_data)
        print(user_status)

        return jsonify({
            'status': 'success',
            'message': '보행자 상태가 업데이트되었습니다',
            'data': pedestrian_data,
            'timestamp': pedestrian_last_updated.isoformat()
        })
        
    except Exception as e:
        return jsonify({
            'error': f'보행자 상태 업데이트 중 오류: {str(e)}'
        }), 500

@app.route('/api/is_ped', methods=['GET'])
def get_pedestrian_status():
    """현재 보행자 상태 조회 API"""
    global user_status
    
    try:
        # 전역변수에서 현재 상태 조회
        with counter_lock:
            current_status = user_status
        
        print(current_status)

        return jsonify({
            'status': 'success',
            'data': current_status,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        return jsonify({
            'error': f'보행자 상태 조회 중 오류: {str(e)}'
        }), 500



# 애플리케이션 컨텍스트 내에서 데이터베이스 초기화
def init_database():
    """데이터베이스 초기화"""
    with app.app_context():
        db.create_all()
        print("데이터베이스 테이블이 생성되었습니다.")
        print("- detection_log: 개별 탐지 이벤트 저장")
        print("- daily_summary: 일별 집계 데이터 저장")

if __name__ == '__main__':
    # 데이터베이스 초기화
    init_database()
    
    
    print("\n🚀 Flask 서버가 시작됩니다...")
    print("\n📍 엔드포인트:")
    print("실시간 카운팅:")
    print("- POST /count : 특정 인덱스 카운트 증가 (value: 0~4)")
    print("- GET /count : 현재 카운트 배열 조회")
    print("- POST /reset : 카운트 배열 리셋 및 집계 저장")
    
    print("\n개별 탐지 로그:")
    print("- POST /detection : 탐지 이벤트 저장 (class_name, object_id, frequency)")
    print("- GET /detections : 탐지 로그 조회 (?class_name=0&date=2024-08-29)")
    
    print("\n통계 및 집계:")
    print("- GET /stats : 클래스별 통계 (?days=7)")
    print("- GET /summary : 일별 집계 조회 (?days=30)")
    print("- GET /summary/2024-08-29 : 특정 날짜 집계 조회")
    
    print("\n시스템:")
    print("- GET /status : 서버 상태 확인")
    print(f"\n💾 데이터베이스: MySQL - {MYSQL_DATABASE}")
    
    # 개발용 서버 실행
    app.run(host='0.0.0.0', port=5000, debug=True)