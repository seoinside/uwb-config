import serial
import time
import csv
from datetime import datetime, timezone, timedelta
import os
   
COM_PORT = 'COM11'       # 본인의 포트로 수정
BAUD_RATE = 921600
NUM = 1

KST = timezone(timedelta(hours=9))

if not os.path.exists('data'):
    os.makedirs('data')

# CSV 저장용 표준 컬럼명 (Full Name)
COLUMNS = [
    'Timestamp', '#Packet', 'ANC', 'PRX', 'RTX', 'FRX', 'ReTX', 
    'FP_idx', 'F1', 'F2', 'F3', 'PP_idx', 'PP_amp', 'N', 'PWR',
    'RSSI', 'FP_PWR', 'Dist', 'FP_Ratio', 'SNR'
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
    current_time_str = datetime.now(KST).strftime('%Y%m%d_%H%M%S_%f')[:-3]
    SAVE_FILENAME = f"data/{NUM}_uwb_{current_time_str}_KST.csv"

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
                    
                    if raw_line.startswith("P:"):
                        parsed_data = parse_uwb_log(raw_line)
                        
                        if '#Packet' in parsed_data:
                            parsed_data['Timestamp'] = datetime.now(KST).strftime('%H:%M:%S.%f')[:-3]
                            writer.writerow(parsed_data)
                            
                            count += 1
                            if count % 1 == 0: 
                                file.flush() 
                                print(f"[{parsed_data['Timestamp']}] PKT:{parsed_data['#Packet']} "
                                      f"Dist:{parsed_data.get('Dist')}m (수집량: {count})")
                                
    except Exception as e:
        print(f"\n에러: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n중지됨. 총 {count}개 데이터 저장 완료.")
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()