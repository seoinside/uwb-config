# uwb-config

DW3000 기반 UWB(Double-Sided Two-Way Ranging, DS-TWR) 설정 코드와 UART 로그 수집 스크립트를 정리한 저장소입니다.

## 프로젝트 구성

```text
.
├── setting/
│   ├── main.c                 # STM32 HAL 초기화 및 예제 실행 진입점
│   ├── ds_twr_initiator.c     # DS-TWR initiator/tag 코드
│   └── ds_twr_responder.c     # DS-TWR responder/anchor 코드
└── logger/
    ├── logger_UTC.py          # 정상 ranging 로그를 CSV로 저장
    └── debug_error.py         # 정상 로그와 에러 로그를 함께 저장
```

## 주요 기능

- DW3000 UWB 칩 초기화 및 SPI 통신 설정
- DS-TWR 방식의 거리 측정
- initiator에서 여러 anchor를 순차적으로 ranging
- responder에서 거리, RSSI, first path power 등 진단값 계산
- UART4를 통한 로그 출력
- Python 스크립트를 이용한 CSV 로그 저장

## 펌웨어 개요

`setting/main.c`는 STM32 HAL 기반으로 GPIO, UART4, I2C1, SPI1, USB CDC를 초기화합니다. 이후 `example_selection.h`에 정의된 매크로에 따라 실행할 예제를 선택합니다.

DS-TWR 예제를 실행하려면 빌드 설정에서 다음 매크로 중 하나를 활성화합니다.

```c
TEST_DS_TWR_INITIATOR
TEST_DS_TWR_RESPONDER
```

### Initiator

`setting/ds_twr_initiator.c`는 tag 역할을 수행합니다.

- anchor ID를 1번부터 `NUM_ANCHORS`까지 순회
- 각 anchor에 Poll 메시지 전송
- Response 수신 후 Final 메시지 전송
- responder가 보낸 Result 메시지를 수신
- 거리와 진단값을 UART/USB CDC 로그로 출력

주요 설정값:

```c
#define NUM_ANCHORS 6
#define MAX_RETRY 5
#define TX_ANT_DLY 16425
#define RX_ANT_DLY 16425
```

### Responder

`setting/ds_twr_responder.c`는 anchor 역할을 수행합니다.

- 자신의 `ANCHOR_ID`와 일치하는 Poll 메시지만 처리
- Response 메시지 전송
- Final 메시지 수신
- DS-TWR 공식으로 거리 계산
- RSSI, first path power, peak 정보 등 진단값을 Result 메시지로 반환

주요 설정값:

```c
#define ANCHOR_ID 2
#define TX_ANT_DLY 16575
#define RX_ANT_DLY 16575
```

anchor를 여러 개 사용할 경우 각 responder 펌웨어에서 `ANCHOR_ID`를 서로 다르게 설정해야 합니다.

## UWB PHY 설정

initiator와 responder는 동일한 PHY 설정을 사용해야 합니다.

현재 코드의 주요 설정은 다음과 같습니다.

| 항목 | 값 |
| --- | --- |
| Channel | 5 |
| Preamble Length | `DWT_PLEN_512` |
| PAC Size | `DWT_PAC16` |
| TX/RX Preamble Code | 9 |
| Data Rate | `DWT_BR_850K` |
| PHR Mode | `DWT_PHRMODE_EXT` |
| STS Mode | `DWT_STS_MODE_OFF` |

## 로그 형식

정상 ranging 결과는 다음과 같은 key-value 형태로 출력됩니다.

```text
P:<cycle>,A:<anchor>,P1:<poll_rx>,R1:<resp_tx>,F1:<final_rx>,S1:<result_tx>,Fi:<fp_idx>,F1:<f1>,F2:<f2>,F3:<f3>,Pi:<peak_idx>,Pa:<peak_amp>,N:<acc_count>,Pw:<power>,R:<rssi>,F:<fp_power>,D:<distance>,Fr:<fp_ratio>,Sn:<snr>
```

에러 로그 예시는 다음과 같습니다.

```text
[A2] [-] RX RESP TIMEOUT
[A2] [-] RX RESULT TIMEOUT
[A2] [-] RX RESULT ERROR
```

## Python 로거 사용법

Python 로거는 UART로 들어오는 UWB 로그를 읽어 CSV 파일로 저장합니다.

필요 패키지:

```bash
pip install pyserial
```

실행 전 `logger/logger_UTC.py` 또는 `logger/debug_error.py`에서 포트 설정을 보드 환경에 맞게 수정합니다.

```python
COM_PORT = 'COM4'
BAUD_RATE = 921600
```

정상 ranging 로그만 저장:

```bash
python logger/logger_UTC.py
```

정상 로그와 에러 로그 함께 저장:

```bash
python logger/debug_error.py
```

CSV 파일은 실행 위치 기준 `data/` 디렉터리에 저장됩니다.

```text
data/uwb_YYYYMMDD_HHMMSS_mmm_UTC.csv
```

## CSV 컬럼

| 컬럼 | 설명 |
| --- | --- |
| `Timestamp` | UTC 기준 수집 시각 |
| `#Packet` | ranging cycle 또는 packet 번호 |
| `ANC` | anchor ID |
| `PRX` | Poll 수신 timestamp |
| `RTX` | Response 송신 timestamp |
| `FRX` | Final 수신 timestamp |
| `ReTX` | Result 송신 timestamp |
| `FP_idx` | first path index |
| `F1`, `F2`, `F3` | first path 진단값 |
| `PP_idx` | peak path index |
| `PP_amp` | peak path amplitude |
| `N` | accumulation count |
| `PWR` | channel power raw value |
| `RSSI` | 추정 RSSI |
| `FP_PWR` | first path power |
| `Dist` | 계산된 거리(m) |
| `FP_Ratio` | first path power와 RSSI 차이 |
| `SNR` | 신호 품질 지표 |
| `Error_Msg` | 에러 로그 메시지 |

## 사용 전 확인 사항

- PC에 연결된 UART 포트가 `COM_PORT`와 일치하는지 확인합니다.
- UART baud rate는 펌웨어와 로거 모두 `921600`으로 맞춥니다.
- initiator와 responder의 UWB PHY 설정이 동일해야 합니다.
- 각 responder의 `ANCHOR_ID`는 고유해야 합니다.
- antenna delay 값은 보드와 안테나 환경에 맞게 보정해야 합니다.
- `debug_error.py`의 `TARGET_ERROR_ANCHORS`를 수정하면 특정 anchor의 에러만 모니터링할 수 있습니다.
