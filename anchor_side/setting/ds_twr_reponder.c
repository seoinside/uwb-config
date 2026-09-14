/*! ----------------------------------------------------------------------------
 * @file    ds_twr_responder.c
 * @brief   Double-sided two-way ranging (DS TWR) responder example code
 * (양방향 거리 측정 방식 중 응답자(앵커) 역할을 하는 예제 코드입니다.)
 */

#define ANCHOR_ID 1  // 현재 이 기기의 앵커 ID (태그가 이 번호를 부를 때만 응답함)

#include "deca_probe_interface.h" // DW3000 칩셋 인터페이스
#include <config_options.h>       // 칩셋 설정
#include <stdio.h>                // 표준 입출력 (sprintf 등)
#include <deca_device_api.h>      // Decawave API
#include <deca_spi.h>             // SPI 통신 API
#include <example_selection.h>
#include <port.h>                 // MCU 포트 제어
#include <shared_defines.h>
#include <shared_functions.h>
#include <stdint.h>
#include <string.h>

#if defined(TEST_DS_TWR_RESPONDER) // 이 예제가 선택되었을 때만 빌드

/* 외부 함수 선언 */
extern void test_run_info(unsigned char *data);
extern uint64_t get_rx_timestamp_u64(void);
extern uint64_t get_tx_timestamp_u64(void);
extern void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);

#define APP_NAME "DS TWR RESP v1.0" // 앱 이름

/* 안테나 지연(Delay) 보정값 (송수신 시 하드웨어 딜레이 보정) */
#define TX_ANT_DLY 16449
#define RX_ANT_DLY 16415

/* ── 송수신 메시지 규격 정의 ── */
// 1. 수신 대기할 Poll 메시지 형식
static uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
		0x21, 0, 0 };
// 2. 응답으로 보낼 Response 메시지 형식
static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
		0x10, 0x02, 0, 0, 0, 0 };
// 3. 마지막으로 수신 대기할 Final 메시지 형식 (24바이트)
static uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
		0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

#define ALL_MSG_COMMON_LEN 10            // 프레임 공통 헤더 길이 (10바이트)
#define ALL_MSG_SN_IDX            2      // 시퀀스 넘버 위치
#define FINAL_MSG_POLL_TX_TS_IDX  10     // Final 메시지 내 Poll TX 시간 위치
#define FINAL_MSG_RESP_RX_TS_IDX  14     // Final 메시지 내 Resp RX 시간 위치
#define FINAL_MSG_FINAL_TX_TS_IDX 18     // Final 메시지 내 Final TX 시간 위치
#define DATA_CUSTOM 			  22     // 커스텀 데이터 위치

static uint8_t frame_seq_nb = 0; // 프레임 시퀀스 번호 (1씩 증가)

#define RX_BUF_LEN 64            // 수신 버퍼 사이즈
static uint8_t rx_buffer[RX_BUF_LEN]; // 수신 데이터를 담을 버퍼

#define CIR_SAMPLE_COUNT 1016
#define CIR_BYTES_PER_SAMPLE 6

// 첫 1바이트는 dummy, 이후 샘플마다 I 3바이트 + Q 3바이트
static uint8_t cir_raw[1 + CIR_SAMPLE_COUNT * CIR_BYTES_PER_SAMPLE];

uint8_t dummy_rng;               // 프레임 길이 읽기용 더미 변수
static uint32_t status_reg = 0;  // 칩셋 상태 레지스터 값 저장 변수

/* 통신 간 지연 시간 설정 (UUS = UWB Microseconds) */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 0   // Response 송신 후 Final 수신 모드 전환 딜레이
#define POLL_RX_TO_RESP_TX_DLY_UUS 2406 // Poll 수신 후 Response 송신까지의 딜레이
#define FINAL_RX_TIMEOUT_UUS 10000         // Final -> Result (timeout)
#define PRE_TIMEOUT 0                   // 프리앰블 타임아웃 끄기

/* 핵심 타임스탬프 3종 보관 변수 */
static uint64_t poll_rx_ts;  // Poll 수신 시간
static uint64_t resp_tx_ts;  // Response 송신 시간
static uint64_t final_rx_ts; // Final 수신 시간

/* 설정 구조체 외부 참조 */
extern dwt_txconfig_t txconfig_options;
static dwt_config_t config_options = { 5, /* Channel number. */
DWT_PLEN_512, /* Preamble length. Used in TX only. */
DWT_PAC16, /* Preamble acquisition chunk size. Used in RX only. */
9, /* TX preamble code. Used in TX only. */
9, /* RX preamble code. Used in RX only. */
1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol... */
DWT_BR_850K, /* Data rate. */
DWT_PHRMODE_EXT, /* PHY header mode. */
DWT_PHRRATE_STD, /* PHY header rate. */
(513 + 8 - 16), /* SFD timeout (preamble length + 1 + SFD length - PAC size). */
DWT_STS_MODE_OFF, /* No STS mode enabled (STS Mode 0). */
DWT_STS_LEN_64, /* STS length */
DWT_PDOA_M0 /* PDOA mode off */
};
extern dwt_txconfig_t txconfig_options_ch9;
//extern dwt_config_t config_options;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ds_twr_responder()
 * @brief 앵커 메인 함수
 */
int ds_twr_responder(void) {
	double tof;        // 비행시간(Time of Flight)
	double distance;   // 최종 계산된 거리

	test_run_info((unsigned char*) APP_NAME); // 앱 이름 출력

	port_set_dw_ic_spi_fastrate(); // SPI 고속 통신 설정
	Sleep(2);

	// DW3000 연결 확인 (프로빙)
	if (dwt_probe((struct dwt_probe_s*) &dw3000_probe_interf) == DWT_ERROR) {
		test_run_info((unsigned char*) "PROBE FAILED");
		while (1) {
		};
	}

	// 칩이 준비 상태(IDLE_RC)가 될 때까지 대기
	while (!dwt_checkidlerc()) {
	};

	// 내부 OTP 메모리 읽어오며 초기화
	if (dwt_initialise(DWT_READ_OTP_ALL) == DWT_ERROR) {
		test_run_info((unsigned char*) "INIT FAILED     ");
		while (1) {
		};
	}

	// 칩셋 옵션 설정
	if (dwt_configure(&config_options)) {
		test_run_info((unsigned char*) "CONFIG FAILED     ");
		while (1) {
		};
	}

	dwt_configciadiag(1); // 상세 채널 진단 모드 활성화 (IPATOV 데이터 등)

	// 채널(5번 or 9번)에 맞는 RF 전력 설정
	if (config_options.chan == 5) {
		dwt_configuretxrf(&txconfig_options);
	} else {
		dwt_configuretxrf(&txconfig_options_ch9);
	}

	// 안테나 지연 적용
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	// LNA, PA 활성화 및 통신 LED 깜빡임 켜기
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
	dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

	uint32_t ranging_samples = 0; // 측정 횟수 카운터

	/* ── 메인 무한 루프 시작 ── */
	while (1) {
		dwt_setpreambledetecttimeout(0); // 프리앰블 탐지 제한 없음
		dwt_setrxtimeout(0);             // 전체 수신 타임아웃 없음 (계속 대기)

		int debug_state = 0; // [디버그용] 현재 통신 상태를 저장할 변수 (루프마다 초기화)

		/* [STEP 1] Poll 메시지 수신 대기 */
		dwt_rxenable(DWT_START_RX_IMMEDIATE); // 즉시 수신 모드 켬

		// 인터럽트 상태 레지스터 폴링 대기
		waitforsysstatus(&status_reg, NULL,
				(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
						| SYS_STATUS_ALL_RX_ERR), 0);

		if (status_reg & DWT_INT_RXFCG_BIT_MASK) // 수신 성공
				{
			uint16_t frame_len;
			dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK); // 플래그 클리어

			// 프레임 길이 확인 후 로컬 버퍼(rx_buffer)로 복사
			frame_len = dwt_getframelength(&dummy_rng);

			// 헤더 10바이트 + anchor ID 1바이트 + FCS 2바이트
			if (frame_len < 13 || frame_len > RX_BUF_LEN) {
			    continue;
			}

			dwt_readrxdata(rx_buffer, frame_len, 0);

			rx_buffer[ALL_MSG_SN_IDX] = 0; // 프레임 비교를 위해 시퀀스 번호 지우기
			// 받은 메시지가 Poll 메시지인지 확인
			if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
				// 메시지 내 태그가 호출한 앵커 ID 확인
				uint8_t target_id = rx_buffer[10];
				if (target_id != ANCHOR_ID) {
					continue; // 나를 부른 게 아니면 무시하고 처음으로
				}

				/* [STEP 2] Response 메시지 송신 */
				uint32_t resp_tx_time;
				int ret;

				poll_rx_ts = get_rx_timestamp_u64(); // Poll 수신 타임스탬프 기록

				// Response 보낼 시간 예약 (현재시간 + 딜레이)
				resp_tx_time = (poll_rx_ts
						+ (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				// Response 전송 후 다시 Final을 수신하기 위한 대기 시간 및 타임아웃 설정
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
				dwt_setpreambledetecttimeout(PRE_TIMEOUT);

				tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb; // 시퀀스 번호 기록
				dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
				dwt_writetxfctrl(sizeof(tx_resp_msg) + 2, 0, 1);

				// 예약 시간에 Response 전송 및 즉시 수신(RX) 모드 전환
				ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

				if (ret == DWT_ERROR) {
					continue; // 예약 전송이 너무 늦어 실패하면 취소
				}

				/* [STEP 3] Final 메시지 수신 대기 */
				waitforsysstatus(&status_reg, NULL,
						(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
								| SYS_STATUS_ALL_RX_ERR), 0);
				frame_seq_nb++; // 다음 전송을 위해 시퀀스 1 증가

				if (status_reg & DWT_INT_RXFCG_BIT_MASK) // Final 수신 성공
				{
				    dwt_writesysstatuslo(
				            DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);

				    frame_len = dwt_getframelength(&dummy_rng);

				    // 타임스탬프와 패킷 번호가 들어갈 길이인지 확인
				    if (frame_len < DATA_CUSTOM + 4 + 2 || frame_len > RX_BUF_LEN) {
				        continue;
				    }

				    dwt_readrxdata(rx_buffer, frame_len, 0);

				    rx_buffer[ALL_MSG_SN_IDX] = 0;

					if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN)
							== 0) // Final이 맞다면
							{
						debug_state = 3; // [디버그용] 상태 3: Final 수신 성공
						/* ── 앵커 자신의 로컬 타임스탬프 갱신 ── */
						resp_tx_ts = get_tx_timestamp_u64(); // 내 Response 송신 시간
						final_rx_ts = get_rx_timestamp_u64(); // 내 Final 수신 시간

						/* ── 거리 계산 (DS-TWR 공식 적용) ── */
						uint32_t poll_tx_ts, resp_rx_ts_local, final_tx_ts; // 태그 측 시간
						uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32; // 앵커 측 시간
						double Ra, Rb, Da, Db;
						uint32_t received_packet_num = 0; // 태그로부터 받은 패킷 번호 저장용

						// Final 메시지 안에서 태그의 타임스탬프들 꺼내기
						final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX],
								&poll_tx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX],
								&resp_rx_ts_local);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX],
								&final_tx_ts);

						// 앵커 64비트 시간을 계산을 위해 32비트로 자름
						poll_rx_ts_32 = (uint32_t) poll_rx_ts;
						resp_tx_ts_32 = (uint32_t) resp_tx_ts;
						final_rx_ts_32 = (uint32_t) final_rx_ts;

						// 둥근 링(Round-trip) 및 지연(Delay) 시간 조각 계산
						Ra = (double) (resp_rx_ts_local - poll_tx_ts);
						Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
						Da = (double) (final_tx_ts - resp_rx_ts_local);
						Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);

						// 비행시간(ToF) 및 빛의 속도 곱해서 최종 거리(m) 산출
						tof = ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db))
								* DWT_TIME_UNITS;
						distance = tof * SPEED_OF_LIGHT;

						// Final에 들어 있는 tag의 패킷 번호 추출
						memcpy(&received_packet_num,
						       &rx_buffer[DATA_CUSTOM],
						       sizeof(received_packet_num));

						// 같은 Final 수신의 CIR과 진단값 읽기
						dwt_readaccdata(cir_raw, (uint16_t)sizeof(cir_raw), 0);

						dwt_rxdiag_t diag;
						memset(&diag, 0, sizeof(diag));
						dwt_readdiagnostics(&diag);

						ranging_samples++;

						char cir_line[384];

						// 1. 측정 시작 + 진단값 출력
						snprintf(cir_line, sizeof(cir_line),
						         "CIR_BEGIN,P:%lu,A:%d,N:%u,D:%.3f,"
						         "FP_RAW:%u,PK_RAW:%lu,ACC:%u,PWR:%lu,"
						         "F1:%lu,F2:%lu,F3:%lu\r\n",
						         (unsigned long)received_packet_num,
						         ANCHOR_ID,
						         (unsigned int)CIR_SAMPLE_COUNT,
						         distance,
						         (unsigned int)diag.ipatovFpIndex,
						         (unsigned long)diag.ipatovPeak,
						         (unsigned int)diag.ipatovAccumCount,
						         (unsigned long)diag.ipatovPower,
						         (unsigned long)diag.ipatovF1,
						         (unsigned long)diag.ipatovF2,
						         (unsigned long)diag.ipatovF3);

						test_run_info((unsigned char*)cir_line);

						// 2. raw I/Q 샘플 출력
						for (uint16_t i = 0; i < CIR_SAMPLE_COUNT; i++) {
						    uint16_t offset = 1 + i * CIR_BYTES_PER_SAMPLE;

						    snprintf(cir_line, sizeof(cir_line),
						             "CIR,%u,%02X%02X%02X%02X%02X%02X\r\n",
						             (unsigned int)i,
						             (unsigned int)cir_raw[offset],
						             (unsigned int)cir_raw[offset + 1],
						             (unsigned int)cir_raw[offset + 2],
						             (unsigned int)cir_raw[offset + 3],
						             (unsigned int)cir_raw[offset + 4],
						             (unsigned int)cir_raw[offset + 5]);

						    test_run_info((unsigned char*)cir_line);
						}

						// 3. 측정 종료 표시
						snprintf(cir_line, sizeof(cir_line),
						         "CIR_END,P:%lu\r\n",
						         (unsigned long)received_packet_num);

						test_run_info((unsigned char*)cir_line);

						debug_state = 4;

					}
				}
				// [디버그 예외]: Final 수신 실패 (타임아웃 또는 에러)
				else if (status_reg & SYS_STATUS_ALL_RX_TO) {
					debug_state = -2; // 상태 -2: Final 수신 타임아웃

					dwt_writesysstatuslo(
							SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
				} else if (status_reg & SYS_STATUS_ALL_RX_ERR) {
					debug_state = -3; // 상태 -3: Final 수신 에러

					dwt_writesysstatuslo(
							SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
				}
			}
		} else {
			// Poll 수신 에러 시
			dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		}
	}
}
#endif
