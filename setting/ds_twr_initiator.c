/*! ----------------------------------------------------------------------------
 * @file    ds_twr_initiator.c
 * @brief   Double-sided two-way ranging (DS TWR) initiator example code
 */

#include "deca_probe_interface.h"
#include <config_options.h>
#include <stdio.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <stdint.h>
#include <string.h>

#if defined(TEST_DS_TWR_INITIATOR)

extern void test_run_info(unsigned char *data);
extern uint64_t get_rx_timestamp_u64(void);
extern uint64_t get_tx_timestamp_u64(void);
extern void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
extern void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);

#define APP_NAME "DS TWR INIT v1.0"

#define RNG_DELAY_MS 20 // 앵커 전체 순회 후 대기 시간 (ms)

#define TX_ANT_DLY 16425 // 송신 안테나 지연값
#define RX_ANT_DLY 16425 // 수신 안테나 지연값

// Poll 메세지 (태그->앵커)
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
		0x21, 0x00, 0, 0 };
// Response 메세지 (앵커->태그)
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
		0x10, 0x02, 0, 0 };
// Final 메세지 (태그->앵커)
static uint8_t tx_final_msg[64] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V',
		'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* 메시지 배열 내 특정 데이터의 위치(인덱스) 정의 */
#define ALL_MSG_COMMON_LEN 10            // 모든 메시지가 공통으로 가지는 헤더 길이 (10바이트)
#define ALL_MSG_SN_IDX            2      // 시퀀스 넘버(Sequence Number)가 들어갈 위치
#define FINAL_MSG_POLL_TX_TS_IDX  10     // Final 메시지 내: Poll 송신 시간 기록 위치
#define FINAL_MSG_RESP_RX_TS_IDX  14     // Final 메시지 내: Response 수신 시간 기록 위치
#define FINAL_MSG_FINAL_TX_TS_IDX 18     // Final 메시지 내: Final 송신 시간 기록 위치
#define DATA_CUSTOM               22     // 커스텀 데이터 기록 위치 (XYZABCDEF 전송용)

#define RESULT_PAYLOAD_LEN  53           // 앵커로부터 받을 Result 데이터 길이
#define RESULT_FRAME_LEN    (RESULT_PAYLOAD_LEN + 2) // 뒤에 붙는 에러 체크용 FCS(2바이트) 포함 길이

/* ── 상태 관리 변수 ── */
static uint8_t frame_seq_nb = 0; // 프레임 시퀀스 번호 (보낼 때마다 1씩 증가하여 메시지 식별)
uint32_t packet_num = 0;         // 전체 통신 시도 횟수를 추적할 패킷 번호
uint8_t current_anchor_id = 1;   // 현재 통신 중인 대상 앵커 ID
#define NUM_ANCHORS 6            // 전체 앵커 개수 (현재 6대의 앵커와 통신)
static uint32_t global_cycle_count = 0; // 주기(Cycle)를 세는 전역 변수 추가
#define MAX_RETRY 5  // 코드 상단 매크로 구역에 추가

#define RX_BUF_LEN 64            // 수신 데이터를 담을 버퍼의 최대 크기
static uint8_t rx_buffer[RX_BUF_LEN]; // 실제 수신 버퍼 배열
uint8_t dummy_rng;               // 프레임 길이 읽기용 더미 변수
static uint32_t status_reg = 0;  // DW3000 하드웨어의 상태 레지스터 값을 저장할 변수

/* ── 송수신 지연(Delay) 및 타임아웃 설정 (UUS = UWB Microseconds 단위) ── */
#define POLL_TX_TO_RESP_RX_DLY_UUS 0       // Poll 전송 후 Response 수신을 시작할 때까지의 대기 시간
#define RESP_RX_TIMEOUT_UUS 10000 // Poll 송신 -> Response 수신 대기 (timeout)
// #define RESP_RX_TO_FINAL_TX_DLY_UUS 506 // 기존 설정
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3000  // Response 수신 후 Final 전송까지의 지연 시간 (계산 시간을 벌어줌)
#define RESULT_RX_TIMEOUT_UUS 10000  // Final 송신 -> Result 수신 대기 (timeout)

/* DS-TWR에 필요한 핵심 타임스탬프 3종을 저장할 변수 */
static uint64_t poll_tx_ts;  // Poll 보낸 시간
static uint64_t resp_rx_ts;  // Response 받은 시간
static uint64_t final_tx_ts; // Final 보낸 시간

/* 외부에서 정의된 칩셋 설정값 가져오기 (채널, PRF, Preamble 길이 등) */
//extern dwt_config_t config_options;
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
extern dwt_txconfig_t txconfig_options;     // 채널 5용 RF 송신 설정
extern dwt_txconfig_t txconfig_options_ch9; // 채널 9용 RF 송신 설정

int ds_twr_initiator(void) {
	char dbg_msg[128];
	static char csv_buf[256];
	port_set_dw_ic_spi_fastrate();
	Sleep(2);

	/* 하드웨어 WAKEUP 핀을 이용해 UWB 칩 강제 깨우기/초기화 */
	HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_SET);
	Sleep(2);
	HAL_GPIO_WritePin(DW_WAKEUP_GPIO_Port, DW_WAKEUP_Pin, GPIO_PIN_RESET);
	Sleep(10);

	// DW3000 칩셋이 정상적으로 연결되어 있는지 ID 확인
	if (dwt_probe((struct dwt_probe_s*) &dw3000_probe_interf) == DWT_ERROR) {
		test_run_info((unsigned char*) "-> PROBE FAILED!!!\r\n");
		while (1) {
			Sleep(1000);
		};
	}

	// DW3000 칩셋이 IDLE_RC (대기) 상태로 진입했는지 확인
	int timeout = 0;
	while (!dwt_checkidlerc()) {
		Sleep(1);
		if (++timeout > 1000)
			break;
	}

	// 칩셋 내부 설정값(OTP 등)을 읽어와서 초기화 진행
	if (dwt_initialise(DWT_READ_OTP_ALL) == DWT_ERROR) {
		test_run_info((unsigned char*) "-> INIT FAILED!!!\r\n");
		while (1) {
			Sleep(1000);
		};
	}

	// 설정된 파라미터(config_options)로 DW3000 동작 모드 세팅
	if (dwt_configure(&config_options)) {
		test_run_info((unsigned char*) "-> CONFIG FAILED!!!\r\n");
		while (1) {
			Sleep(1000);
		};
	}

	if (config_options.chan == 5) {
		dwt_configuretxrf(&txconfig_options);
	} else {
		dwt_configuretxrf(&txconfig_options_ch9);
	}

	// 계산된 안테나 지연(Delay) 값 하드웨어에 등록
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	// 송신 후 수신 활성화까지의 지연 시간 및 수신 타임아웃 설정
	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	dwt_setpreambledetecttimeout(0);

	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
	dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

	/* ★ 추가: 초기화 성공 확인 로그 */
	    test_run_info((unsigned char*)"-> [SUCCESS] DW3000 Initialization Complete!\r\n");
	    test_run_info((unsigned char*)"-> Starting Ranging Loop...\r\n");

	while (1) {
		global_cycle_count++; // 새로운 주기 시작 (6개 앵커가 이 번호를 공유)

		for (current_anchor_id = 1; current_anchor_id <= NUM_ANCHORS;
				current_anchor_id++) {

			// --- 리트라이 루프 시작 ---
			for (int retry = 0; retry < MAX_RETRY; retry++) {
				int debug_state = 0; // [디버그용] 현재 통신 상태를 저장할 변수

				/* [STEP 1] Poll 메시지 송신 */
				tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				tx_poll_msg[10] = current_anchor_id;
				dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
				dwt_writetxfctrl(sizeof(tx_poll_msg) + 2, 0, 1);

				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				// Response 수신 대기 (20ms)
				waitforsysstatus(&status_reg, NULL,
						(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
								| SYS_STATUS_ALL_RX_ERR), RESP_RX_TIMEOUT_UUS);
				frame_seq_nb++;

				if (status_reg & DWT_INT_RXFCG_BIT_MASK) {

					uint16_t f_len = dwt_getframelength(&dummy_rng);
					if (f_len <= RX_BUF_LEN) dwt_readrxdata(rx_buffer, f_len, 0);
					dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK); // 데이터를 다 읽은 후 수신 플래그만 클리어

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN)
							== 0) {

						/* [STEP 3] Final 메시지 송신 */
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();
						uint32_t final_tx_time =
								(uint32_t) ((resp_rx_ts
										+ (RESP_RX_TO_FINAL_TX_DLY_UUS
												* UUS_TO_DWT_TIME)) >> 8);
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts =
								(((uint64_t) (final_tx_time & 0xFFFFFFFEUL))
										<< 8) + TX_ANT_DLY;

						final_msg_set_ts(
								&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX],
								poll_tx_ts);
						final_msg_set_ts(
								&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX],
								resp_rx_ts);
						final_msg_set_ts(
								&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX],
								final_tx_ts);
						tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

						packet_num++; // 태그 측 글로벌 카운트 (필요 시 사용)
						memcpy(&tx_final_msg[DATA_CUSTOM], &packet_num, 4);

						dwt_writetxdata(31, tx_final_msg, 0);
						dwt_writetxfctrl(31 + 2, 0, 1);

						if (dwt_starttx(
								DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED)
								== DWT_SUCCESS) {
							waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
							dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
							frame_seq_nb++;

							// Result 수신 대기 (30ms)
							waitforsysstatus(&status_reg, NULL,
									(DWT_INT_RXFCG_BIT_MASK
											| SYS_STATUS_ALL_RX_TO
											| SYS_STATUS_ALL_RX_ERR), RESULT_RX_TIMEOUT_UUS);

							/* [STEP 4] Result 수신 및 파싱 */
							if (status_reg & DWT_INT_RXFCG_BIT_MASK) {

								uint16_t res_len = dwt_getframelength(
										&dummy_rng);
								if (res_len == RESULT_FRAME_LEN) {
									dwt_readrxdata(rx_buffer, res_len, 0);

									// 데이터 파싱
									uint8_t off = 0;
									uint8_t a_id = rx_buffer[off++];
									uint32_t p1, r1, f1, s1;
									memcpy(&p1, &rx_buffer[off], 4);
									off += 4;
									memcpy(&r1, &rx_buffer[off], 4);
									off += 4;
									memcpy(&f1, &rx_buffer[off], 4);
									off += 4;
									memcpy(&s1, &rx_buffer[off], 4);
									off += 4;

									uint16_t fi, ac;
									uint32_t v1, v2, v3, pk, pw;
									memcpy(&fi, &rx_buffer[off], 2);
									off += 2;
									memcpy(&v1, &rx_buffer[off], 4);
									off += 4;
									memcpy(&v2, &rx_buffer[off], 4);
									off += 4;
									memcpy(&v3, &rx_buffer[off], 4);
									off += 4;
									memcpy(&pk, &rx_buffer[off], 4);
									off += 4;
									memcpy(&pw, &rx_buffer[off], 4);
									off += 4;
									memcpy(&ac, &rx_buffer[off], 2);
									off += 2;

									float rssi, fpp, dist;
									memcpy(&rssi, &rx_buffer[off], 4);
									off += 4;
									memcpy(&fpp, &rx_buffer[off], 4);
									off += 4;
									memcpy(&dist, &rx_buffer[off], 4);
									off += 4;

									debug_state = 4; // [디버그용] 모든 통신 및 파싱 성공

									sprintf(csv_buf,
											"P:%lu,A:%d,P1:%lu,R1:%lu,F1:%lu,S1:%lu,Fi:%u,F1:%lu,F2:%lu,F3:%lu,Pi:%u,Pa:%lu,N:%u,Pw:%lu,R:%.1f,F:%.1f,D:%.3f,Fr:%.1f,Sn:%.1f\r\n",
											global_cycle_count, a_id, p1, r1,
											f1, s1, fi, v1, v2, v3,
											(uint16_t) ((pk >> 21) & 0x3FFu),
											(uint32_t) (pk & 0x1FFFFFu), ac, pw,
											rssi, fpp, dist, fpp - rssi,
											fpp - rssi);

									dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
								} else {
									debug_state = -7;
								}
							}
							else if (status_reg & SYS_STATUS_ALL_RX_TO)
								debug_state = -4; // Result Timeout
							else if (status_reg & SYS_STATUS_ALL_RX_ERR)
								debug_state = -5; // Result Error
						} else {
							debug_state = -3; // Final 송신 딜레이 오류
						}
					}
				} else if (status_reg & SYS_STATUS_ALL_RX_TO)
					debug_state = -1; // Response Timeout
				else if (status_reg & SYS_STATUS_ALL_RX_ERR)
					debug_state = -2; // Response Error

				if (debug_state == 4) {
					test_run_info((unsigned char*) csv_buf);
				    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
				    goto next_anchor;
				} else {
					// 2. 실패했을 경우 원인 출력 및 리트라이 준비
					char err_msg[64] = {0};
					if (debug_state == -1)
						sprintf(err_msg, "[A%d] [-] RX RESP TIMEOUT\r\n",
								current_anchor_id);
					else if (debug_state == -2)
						sprintf(err_msg, "[A%d] [-] RX RESP ERROR\r\n",
								current_anchor_id);
					else if (debug_state == -3)
						sprintf(err_msg, "[A%d] [-] TX FINAL LATE ERROR\r\n",
								current_anchor_id);
					else if (debug_state == -4)
						sprintf(err_msg, "[A%d] [-] RX RESULT TIMEOUT\r\n",
								current_anchor_id);
					else if (debug_state == -5)
						sprintf(err_msg, "[A%d] [-] RX RESULT ERROR\r\n",
								current_anchor_id);
					else if (debug_state == -7)
					    sprintf(err_msg, "[A%d] [-] RESULT LENGTH MISMATCH!\r\n", current_anchor_id);
					else if (debug_state == 0)
										    sprintf(err_msg, "something error\r\n", current_anchor_id);

					test_run_info((unsigned char*) err_msg);

					// 실패했을 경우: 실패한 앵커만 retry 루프에 의해 다시 시도
					dwt_writesysstatuslo(
							SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_forcetrxoff();
				}
			} // Retry 루프 끝
			next_anchor: ;
			//Sleep(3);
		}
		//Sleep(RNG_DELAY_MS);
	}
}
#endif
