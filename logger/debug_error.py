import serial
import csv
from datetime import datetime, timezone
import os
import re

COM_PORT = 'COM4'       # 본인의 포트로 수정
BAUD_RATE = 921600

# 에러 로그를 확인할 특정 앵커 번호 설정 
# 예: [3] -> 3번 에러만 표시 / [1, 3] -> 1번, 3번 에러 표시 / [] -> 모든 에러 표시
TARGET_ERROR_ANCHORS = [2]
if not os.path.exists('data'):
    os.makedirs('data')

# CSV 저장용 표준 컬럼명
COLUMNS = [
    'Timestamp', '#Packet', 'ANC', 'PRX', 'RTX', 'FRX', 'ReTX', 
    'FP_idx', 'F1', 'F2', 'F3', 'PP_idx', 'PP_amp', 'N', 'PWR',
    'RSSI', 'FP_PWR', 'Dist', 'FP_Ratio', 'SNR', 'Error_Msg'
]

# 축약 키 -> 표준 컬럼 매핑 사전
KEY_MAP = {
    'P': '#Packet', 'A': 'ANC', 'P1': 'PRX', 'R1': 'RTX',
    'F1': 'FRX',  
    'S1': 'ReTX', 'Fi': 'FP_idx', 'F2': 'F2', 'F3': 'F3',
    'Pi': 'PP_idx', 'Pa': 'PP_amp', 'N': 'N', 'Pw': 'PWR',
    'R': 'RSSI', 'F': 'FP_PWR', 'D': 'Dist', 'Fr': 'FP_Ratio', 'Sn': 'SNR'
}

def parse_uwb_log(line):
    parts = line.replace(' ', '').split(',')
    data = {}
    f1_count = 0
    
    for part in parts:
        if ':' not in part: continue
        k, v = part.split(':', 1)
        
        if k == 'F1':
            if f1_count == 0:
                real_key = 'FRX'
                f1_count += 1
            else:
                real_key = 'F1'
        else:
            real_key = KEY_MAP.get(k, k)
        
        if real_key not in COLUMNS: continue

        try:
            if real_key in ['RSSI', 'FP_PWR', 'Dist', 'FP_Ratio', 'SNR']:
                data[real_key] = float(v)
            elif real_key == 'FP_idx':
                data[real_key] = float(v) / 64.0 
            else:
                data[real_key] = int(v)
        except ValueError:
            continue
            
    return data

def main():
    current_time_str = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')[:-3]
    SAVE_FILENAME = f"data/uwb_{current_time_str}_UTC.csv"

    print(f"UWB 로그 수집 시작  (Baud:{BAUD_RATE})")
    print(f"저장 경로: {SAVE_FILENAME}") 
    if TARGET_ERROR_ANCHORS:
        print(f"에러 모니터링 앵커: {TARGET_ERROR_ANCHORS}번")
    else:
        print(f"에러 모니터링 앵커: 전체 모니터링")
    
    count = 0
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)
        print(f"{COM_PORT} 연결 성공. (중단: Ctrl+C)")
        
        with open(SAVE_FILENAME, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=COLUMNS)
            writer.writeheader()
            
            while True:
                if ser.in_waiting > 0:
                    raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not raw_line:
                        continue
                    
                    # 1. 정상 데이터 수신 처리 (P: 로 시작)
                    if raw_line.startswith("P:"):
                        parsed_data = parse_uwb_log(raw_line)
                        
                        if '#Packet' in parsed_data:
                            parsed_data['Timestamp'] = datetime.now(timezone.utc).strftime('%H:%M:%S.%f')[:-3]
                            writer.writerow(parsed_data)
                            
                            count += 1
                            if count % 1 == 0: 
                                file.flush() 
                                print(f"[{parsed_data['Timestamp']}] PKT:{parsed_data['#Packet']} "
                                      f"ANC:{parsed_data.get('ANC')} Dist:{parsed_data.get('Dist')}m (수집량: {count})")
                    
                    # 2. 예외 로그 및 에러 메시지 처리
                    else:
                        if "[-]" in raw_line:
                            match = re.search(r'\[A(\d+)\]', raw_line)
                            
                            if match:
                                error_anc = int(match.group(1))
                                
                                # 리스트가 비어있거나([]), 현재 에러 앵커 번호가 리스트 안에 있을 때만 실행
                                if not TARGET_ERROR_ANCHORS or error_anc in TARGET_ERROR_ANCHORS:
                                    print(f"\n[ERROR LOG] {raw_line}")
                                    
                                    error_time = datetime.now(timezone.utc).strftime('%H:%M:%S.%f')[:-3]
                                    error_data = {
                                        'Timestamp': error_time,
                                        'ANC': error_anc,
                                        'Error_Msg': raw_line
                                    }
                                    writer.writerow(error_data)
                                    file.flush()
                                    print(f"[A{error_anc}] 에러 내역 CSV 저장")
                            
                            else:
                                # 앵커 번호가 명시되지 않은 에러 처리 (전체 모니터링일 때만 표시)
                                if not TARGET_ERROR_ANCHORS:
                                    print(f"\n[ERROR LOG] {raw_line}")
                                    error_time = datetime.now(timezone.utc).strftime('%H:%M:%S.%f')[:-3]
                                    error_data = {
                                        'Timestamp': error_time,
                                        'Error_Msg': raw_line
                                    }
                                    writer.writerow(error_data)
                                    file.flush()
                                    print(f"[알 수 없는 앵커] 에러 내역 CSV 저장")
                                
                        else:
                            print(f"[SYS LOG] {raw_line}")
                                
    except Exception as e:
        print(f"\n에러: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        print("=" * 50)
        print(f"중지: 총 {count}개 정상 데이터 저장 완료")
        print("=" * 50)
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()