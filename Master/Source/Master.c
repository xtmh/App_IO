/****************************************************************************
 * (C) Tokyo Cosmos Electric, Inc. (TOCOS) - 2013 all rights reserved.
 *
 * Condition to use: (refer to detailed conditions in Japanese)
 *   - The full or part of source code is limited to use for TWE (TOCOS
 *     Wireless Engine) as compiled and flash programmed.
 *   - The full or part of source code is prohibited to distribute without
 *     permission from TOCOS.
 *
 * 利用条件:
 *   - 本ソースコードは、別途ソースコードライセンス記述が無い限り東京コスモス電機が著作権を
 *     保有しています。
 *   - 本ソースコードは、無保証・無サポートです。本ソースコードや生成物を用いたいかなる損害
 *     についても東京コスモス電機は保証致しません。不具合等の報告は歓迎いたします。
 *   - 本ソースコードは、東京コスモス電機が販売する TWE シリーズ上で実行する前提で公開
 *     しています。他のマイコン等への移植・流用は一部であっても出来ません。
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <string.h>
#include <AppHardwareApi.h>

#include "Master.h"

#include "ccitt8.h"
#include "Interrupt.h"

#include "Version.h"

#include "utils.h"
#include "flash.h"

#include "common.h"
#include "config.h"

// IO Read Options
#include "btnMgr.h"

// 重複チェッカ
#include "duplicate_checker.h"

// Serial options
#include <serial.h>
#include <fprintf.h>
#include <sprintf.h>

#include "modbus_ascii.h"
#include "input_string.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
// Select Modules (define befor include "ToCoNet.h")
// #define ToCoNet_USE_MOD_RXQUEUE_BIG
#define ToCoNet_USE_MOD_CHANNEL_MGR

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#undef QUICK_TRANSFER_ON_MODE7
#ifdef QUICK_TRANSFER_ON_MODE7
// 実験的なモード。
// MODE7 で電源が入った時に、IO状態を見ず HoldMask にあるポートを押し下げとしてパケット送信する
# warning "QUICK_TRANSFER_ON_MODE7"
#endif

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vProcessEvCoreSlp(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);
static void vProcessEvCorePwr(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);

static void vInitHardware(int f_warm_start);
static void vInitHardware_IOs(int f_warm_start);

static void vChangeChannelPreset(uint8 u8preset);

static void vSerialInit(uint32, tsUartOpt *);
static void vProcessSerialCmd(tsModbusCmd *pSer);
static void vProcessInputByte(uint8 u8Byte);
static void vProcessInputString(tsInpStr_Context *pContext);
static void vHandleSerialInput();
static void vSerUpdateScreen();
static void vSerInitMessage();

static void vReceiveIoData(tsRxDataApp *pRx);
static void vReceiveIoSettingRequest(tsRxDataApp *pRx);
static void vReceiveSerialMsg(tsRxDataApp *pRx);

static bool_t bCheckDupPacket(tsDupChk_Context *pc, uint32 u32Addr, uint16 u16TimeStamp);

static int16 i16TransmitIoData(bool_t bQuick);
static int16 i16TransmitIoSettingRequest(uint8 u8DstAddr, tsIOSetReq *pReq);
static int16 i16TransmitSerMsg(uint8 u8DstAddr, uint8 u8Cmd, uint8 *pDat, uint8 u8len);
static int16 i16TransmitRepeat(tsRxDataApp *pRx);

static void vConfig_SetDefaults(tsFlashApp *p);
static void vConfig_UnSetAll(tsFlashApp *p);
static void vConfig_SaveAndReset();

static void vSleep(uint32 u32SleepDur_ms, bool_t bPeriodic, bool_t bDeep, bool_t bNoIoInt);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
static tsAppData sAppData; //!< アプリケーションデータ  @ingroup MASTER
static uint32 u32WakeBtnStatus; //!< 起床時のボタンの状態 @ingroup MASTER
static uint16 u16OnPrsTx_Count; //!< リモコンモードにおける余剰送信回数のカウント @ingroup MASTER
static bool_t bChatteringSleep; //!< TRUEなら、チャタリング抑制を目的としたスリープを実施した @ingroup MASTER

tsFILE sSerStream; //!< シリアル出力ストリーム  @ingroup MASTER
tsSerialPortSetup sSerPort; //!< シリアルポートの設定  @ingroup MASTER

tsModbusCmd sSerCmdIn; //!< シリアル入力系列のパーサー (modbus もどき)  @ingroup MASTER
tsInpStr_Context sSerInpStr; //!< 文字列入力  @ingroup MASTER
static uint16 u16HoldUpdateScreen = 0; //!< スクリーンアップデートを行う遅延カウンタ  @ingroup MASTER

tsTimerContext sTimerApp; //!< タイマー管理構造体  @ingroup MASTER

uint8 au8SerInBuff[128]; //!< シリアルの入力書式のための暫定バッファ  @ingroup MASTER
uint8 au8SerOutBuff[256]; //!< シリアルの出力書式のための暫定バッファ  @ingroup MASTER

tsDupChk_Context sDupChk_IoData; //!< 重複チェック(IO関連のデータ転送)  @ingroup MASTER
tsDupChk_Context sDupChk_SerMsg; //!< 重複チェック(シリアル関連のデータ転送)  @ingroup MASTER

uint32 u32_PORT_INPUT_MASK; //!< 入力ポートのマスク @ingroup MASTER

/****************************************************************************/
/***        FUNCTIONS                                                     ***/
/****************************************************************************/

/** @ingroup MASTER
 * アプリケーションの基本制御状態マシン。
 * - 特別な処理は無い。
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch (pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			if (IS_APPCONF_ROLE_SILENT_MODE()) {
				vfPrintf(&sSerStream, LB"!Note: launch silent mode."LB);
				ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
			} else {
				// LayerNetwork で無ければ、特別な動作は不要。

				// 暗号化鍵の設定
				if (IS_CRYPT_MODE()) {
					ToCoNet_bRegisterAesKey((void*)(sAppData.sFlash.sData.au8AesKey), NULL);
				}

				// 始動メッセージの表示
				if (!(u32evarg & EVARG_START_UP_WAKEUP_MASK)) {
					vSerInitMessage();
				}

				// RUNNING 状態へ遷移
				ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
			}

			break;
		}

		break;

	case E_STATE_RUNNING:
		break;
	default:
		break;
	}
}

/** @ingroup MASTER
 * アプリケーション制御（電源常時 ON モード）
 * - 機能概要
 *   - 起動時にランダムで処理を保留する（同時起動による送信パケットの競合回避のため）
 *   - 初回のDI/AD状態確定まで待つ
 *   - 実行状態では E_EVENT_APP_TICK_A (64fps タイマーイベント) を起点に処理する。
 *     - 32fps のタイミングで送信判定を行う
 *     - 定期パケット送信後は、次回のタイミングを乱数によってブレを作る。
 *
 * - 状態一覧
 *   - E_STATE_IDLE\n
 *     起動直後に呼び出される状態で、同時起動によるパケット衝突を避けるためランダムなウェイトを置き、次の状態に遷移する。
 *   - E_STATE_APP_WAIT_IO_FIRST_CAPTURE\n
 *     初回に DI および AI の入力値が確定するまでの待ちを行い、E_STATE_RUNNING に遷移する。
 *   - E_STATE_RUNNING
 *     秒６４回のタイマー割り込み (E_EVENT_TICK_A) を受けて、入力状態の変化のチェックを行い、無線パケットの送信要求を
 *     発行する。各種判定条件があるので、詳細はコード中のコメント参照。
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
void vProcessEvCorePwr(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {

	switch(pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			sAppData.u16CtRndCt = 0;
		}

		if (eEvent == E_EVENT_TICK_TIMER) {
			static bool_t bStarted = FALSE;

			if (!sAppData.u16CtRndCt) {
				bStarted = TRUE;
				sAppData.u16CtRndCt = (ToCoNet_u16GetRand() & 0xFF) + 10; // 始動時にランダムで少し待つ（同時電源投入でぶつからないように）
			}
		}

		// 始動時ランダムな待ちを置く
		if (sAppData.u16CtRndCt && PRSEV_u32TickFrNewState(pEv) > sAppData.u16CtRndCt) {
			if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_RUNNING_PARENT);
			} else if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_IO_FIRST_CAPTURE);
			} else {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_RUNNING_ROUTER);
			}
			sAppData.u16CtRndCt = 32; // この変数は定期送信のタイミング用に再利用する。
		}

		break;

	case E_STATE_APP_WAIT_IO_FIRST_CAPTURE:
		// 起動直後の未確定状態
		if (eEvent == E_EVENT_APP_TICK_A) {
			if (sAppData.u8IOFixState == E_IO_FIX_STATE_READY) {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_RUNNING_CHILD);
			} else {
				;
			}
		}
		break;

	case E_STATE_APP_RUNNING_ROUTER: // 中継機
		break;

	case E_STATE_APP_RUNNING_PARENT: // 親機
		// 一定期間受信が無い場合、IO状態をHIに自動で戻す処理
		if (IS_APPCONF_OPT_ON_PRESS_TRANSMIT()) {
			if (u32TickCount_ms - sAppData.sIOData_now.u32RxLastTick > sAppData.sFlash.sData.u16HoldDur_ms) { // 一定時間で自動停止
				int i = 0, j = 1;

				// ポートの値を設定する（変更フラグのあるものだけ）
				for (; i < sAppData.u8MaxIoCount; i++, j <<= 1) {
					if (j & sAppData.sFlash.sData.u32HoldMask) {
						vPortSetHi(au8PortTbl_DOut[i]);
						sAppData.sIOData_now.au8Output[i] = 0;
					}
				}
			}
		}
		break;

	case E_STATE_APP_RUNNING_CHILD: // 子機
		if (eEvent == E_EVENT_APP_TICK_A // 秒64回のタイマー割り込み
			&& (sAppData.u32CtTimer0 & 1) // 秒32回にする
			) {
			// 変更が有った場合は送信する
			//int i;

			if (sAppData.u16CtRndCt) sAppData.u16CtRndCt--; // 定期パケット送信までのカウントダウン

			bool_t bTxCond = FALSE; // 送信条件

			// リモコンモードにおける送信条件(長押しの判定)
			if (   IS_APPCONF_OPT_ON_PRESS_TRANSMIT()
				&& ((sAppData.u32CtTimer0 & sAppData.u8FpsBitMask) == sAppData.u8FpsBitMask)
				&& (sAppData.u8IOFixState == E_IO_FIX_STATE_READY)
			) {
				if (sAppData.sIOData_now.u32BtmBitmap & sAppData.u32BtnMask_Special) {
					// 長押し対象のボタンを検出した

					static uint16 u16CountMax = 0;
					if (!u16CountMax) {
						// 4fps: 1111           = 256ms
						// 8fps:  111 (64/8 -1) = 128ms
						// 16pfs:  11 (64/16-1) = 64ms
						// 32fps:   1 (64/32-1) = 32ms
						u16CountMax = sAppData.sFlash.sData.u16HoldDur_ms / ((sAppData.u8FpsBitMask + 1) * 16) + 1;
					}

					u16OnPrsTx_Count = u16CountMax;
				}

				if (u16OnPrsTx_Count) {
					u16OnPrsTx_Count--;
				}

				if (u16OnPrsTx_Count) {//TODO
					// 送信する
					bTxCond |= TRUE;
				}
			}

			// リモコンモード時に長押しを検知した場合は、他のボタンの入力を無視する
			if (u16OnPrsTx_Count
				&& (IS_APPCONF_OPT_LOW_LATENCY_INPUT() // 低レイテンシの場合
				    || (((sAppData.sIOData_now.u32BtmChanged & u32_PORT_INPUT_MASK) & (~sAppData.u32BtnMask_Special)) == 0)) // 非長押しボタン変化なし
			) {
				// 関係ないポートをクリア
				int i;
				for (i = 0; i < sAppData.u8MaxIoCount; i++) {
					if (!(sAppData.sFlash.sData.u32HoldMask & (1UL << i))) { // 押し下げを検知したポート
						sAppData.sIOData_now.au8Input[i] = 0;
					}
				}
				sAppData.sIOData_now.u32BtmBitmap &= sAppData.u32BtnMask_Special;
				sAppData.sIOData_now.u32BtmUsed &= sAppData.u32BtnMask_Special;
			}

			// IOの変化により送信する
			if (IS_APPCONF_OPT_ON_PRESS_TRANSMIT() && IS_APPCONF_OPT_LOW_LATENCY_INPUT()) {
				// 低レイテンシとリモコンモードでの通常ボタンは割り込み検出のみで送信する
			} else {
				// IO変化あり
				bTxCond |= sAppData.sIOData_now.u32BtmChanged ? TRUE : FALSE;
			}

			// 定期送信
			if ((sAppData.u16CtRndCt == 0) && !IS_APPCONF_OPT_ON_PRESS_TRANSMIT()) {
				if (!IS_APPCONF_OPT_NO_REGULAR_TX()) {
					bTxCond |= TRUE;
				}
			}

			// 連続送信モード
			if (sAppData.u8Mode == E_IO_MODE_CHILD_CONT_TX
						&& ((sAppData.u32CtTimer0 & sAppData.u8FpsBitMask) == sAppData.u8FpsBitMask)) {
				bTxCond |= TRUE;
			}

			// 条件が整ったので送信する
			if (bTxCond) {
				// デバッグ出力
				DBGOUT(5, "A(%02d/%04d) %d: B=%d%d%d%d %08x"LB,
					sAppData.u32CtTimer0,
					u32TickCount_ms & 8191,
					sAppData.sIOData_now.u32BtmChanged ? 1 : 0,
					sAppData.sIOData_now.au8Input[0] & 1,
					sAppData.sIOData_now.au8Input[1] & 1,
					sAppData.sIOData_now.au8Input[2] & 1,
					sAppData.sIOData_now.au8Input[3] & 1,
					sAppData.sIOData_now.u32BtmBitmap
				);

				// 低遅延送信が必要かどうかの判定
				bool_t bQuick = FALSE;
				if (sAppData.sIOData_now.u32BtmChanged && (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT)) {
					bQuick = TRUE;
				}

				// 送信要求
				sAppData.sIOData_now.i16TxCbId = i16TransmitIoData(bQuick);

				// 変更フラグのクリア
				sAppData.sIOData_now.u32BtmChanged = 0;

				// 次の定期パケットのタイミングを仕込む
				sAppData.u16CtRndCt = (ToCoNet_u16GetRand() & 0xF) + 24;
			}
		}
		break;

	default:
		break;
	}
}

/**  @ingroup MASTER
 * アプリケーション制御（スリープ稼動モード）\n
 * 本状態遷移マシンは、mode=4, mode=7 で起動したときに登録され、測定完了待ち⇒送信⇒
 * 送信完了待ちを実施し、その後、再びスリープを実行する。
 *
 * - 機能概要
 *   - ADやDIの状態が確定するまで待つ。
 *   - 送信する。
 *   - 送信完了を待つ。
 *   - スリープする。
 *
 * - 状態一覧
 *   - E_STATE_IDLE\n
 *     起動直後またはスリープ復帰後の状態。UARTにメッセージを表示し、最初の TickTimer で
 *     E_STATE_RUNNING に遷移する。
 *   - E_STATE_RUNNING\n
 *     IO状態の確定を待って、無線送信要求、E_STATE_WAIT_TX へ遷移。
 *   - E_STATE_WAIT_TX\n
 *     送信完了イベントを待つ。実際は cbToCoNet_TxEvent() よりコールされる。
 *   - E_STATE_FINISHED\n
 *     スリープ条件が成立するまでの待ちを行う。具体的にはボタン駆動した時にチャタリングの
 *     影響が去るまでの時間待ちである。
 *   - E_STATE_APP_SLEEPING\n
 *     スリープ処理を行う。
 *
 * @param pEv
 * @param eEvent
 * @param u32evarg
 */
void vProcessEvCoreSlp(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
	switch(pEv->eState) {
	case E_STATE_IDLE:
		if (eEvent == E_EVENT_START_UP) {
			// vfPrintf(&sSerStream, "START_UP"LB, eEvent);
			if (u32evarg & EVARG_START_UP_WAKEUP_MASK) {
				vfPrintf(&sSerStream, "!INF %s WAKE UP."LB, sAppData.bWakeupByButton ? "DI" : "TIMER");
			}
		}
		ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);
		break;

	case E_STATE_RUNNING:

		// 割り込み要因に対して送信する。
		if (eEvent == E_EVENT_NEW_STATE
				&& sAppData.bWakeupByButton // IO割り込みで起床
				&& IS_APPCONF_OPT_LOW_LATENCY_INPUT_SLEEP_TX_BY_INT()) { // オプションビットの設定
			int i;

			vfPrintf(&sSerStream, "!INF DI(INT):");
			for (i = 0; i < sAppData.u8MaxIoCount; i++) {
				vfPrintf(&sSerStream, "%c", u32WakeBtnStatus & (1UL << au8PortTbl_DIn[i]) ? '1' : '0');

				// 入力状態の情報を構築
				sAppData.sIOData_now.au8Input[i] = u32WakeBtnStatus & (1UL << au8PortTbl_DIn[i]) ? 1 : 0;
			}
			vfPrintf(&sSerStream, LB);

			sAppData.sIOData_now.u32BtmBitmap = u32WakeBtnStatus & u32_PORT_INPUT_MASK;
			sAppData.sIOData_now.u32BtmUsed = u32WakeBtnStatus & u32_PORT_INPUT_MASK;

			ToCoNet_Event_SetState(pEv, E_STATE_APP_TX);
			break;
		}

#ifdef QUICK_TRANSFER_ON_MODE7
		// HOLD マスクにあるポートを押されたとして速やかに送信する
		if (eEvent == E_EVENT_NEW_STATE) {
			int i;
			sAppData.sIOData_now.u32BtmBitmap = 0;
			for (i = 0; i < sAppData.u8MaxIoCount; i++) {
				if (sAppData.sFlash.sData.u32HoldMask & (1UL << i)) { // 押し下げを検知したポート
					sAppData.sIOData_now.au8Input[i] = 1;
					sAppData.sIOData_now.u32BtmBitmap |= au8PortTbl_DIn[i];
				} else {
					sAppData.sIOData_now.au8Input[i] = 0;
				}
			}
			sAppData.sIOData_now.u32BtmUsed = sAppData.sIOData_now.u32BtmBitmap;

			if (sAppData.sIOData_now.u32BtmBitmap) {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_TX);
				break;
			}
		}
#endif

		// IO状態が確定すれば送信する。
		if (sAppData.u8IOFixState == E_IO_FIX_STATE_READY) {
			int i;
			vfPrintf(&sSerStream, "!INF DI:");
			for (i = 0; i < sAppData.u8MaxIoCount; i++) {
				vfPrintf(&sSerStream, "%d", sAppData.sIOData_now.au8Input[i] & 1);
			}
			vfPrintf(&sSerStream, LB);

			ToCoNet_Event_SetState(pEv, E_STATE_APP_TX);
		}

		// 恐らくここに迷い込むことはないのだが念のためタイムアウトを設定する
		if (PRSEV_u32TickFrNewState(pEv) > 100) {
			vfPrintf(&sSerStream, "!INF TimeOut (E_STATE_RUNNING)");
			vWait(10000);
			vAHI_SwReset();
		}
		break;

	case E_STATE_APP_TX:
		if (eEvent == E_EVENT_NEW_STATE) {
			bool_t bRemoteMode = FALSE;

			// 長押しボタンのいずれかが押されている
			if (IS_APPCONF_OPT_ON_PRESS_TRANSMIT()
				&& (sAppData.sIOData_now.u32BtmBitmap & sAppData.u32BtnMask_Special)) {
				bRemoteMode = TRUE;

				u16OnPrsTx_Count = REMOTE_MODE_ADDITIONAL_TX_COUNT; // 長押しボタンがリリースされてからの検出回数
			}

			// 直前のスリープで長押しボタンが押された
			if (u16OnPrsTx_Count) {
				bRemoteMode = TRUE;

				// 長押し中のスリープ回復時は他通常ボタンの状態はクリアする
				if (!(sAppData.bWakeupByButton && IS_APPCONF_OPT_LOW_LATENCY_INPUT_SLEEP_TX_BY_INT())) {
					int i;
					for (i = 0; i < sAppData.u8MaxIoCount; i++) {
						if (!(sAppData.sFlash.sData.u32HoldMask & (1UL << i))) { // 押し下げを検知したポート
							sAppData.sIOData_now.au8Input[i] = 0;
						}
					}
					sAppData.sIOData_now.u32BtmBitmap &= sAppData.u32BtnMask_Special;
					sAppData.sIOData_now.u32BtmUsed &= sAppData.u32BtnMask_Special;
				}
			}

			// クイックで送信。完了待ちをするため CbId を保存する。
			sAppData.sIOData_now.i16TxCbId = i16TransmitIoData(TRUE);

			if (bRemoteMode) {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_REMOTE_WAIT_TX);
			} else {
				ToCoNet_Event_SetState(pEv, E_STATE_WAIT_TX);
			}
		}
		break;

	case E_STATE_APP_REMOTE_WAIT_TX:
		if (eEvent == E_EVENT_APP_TX_COMPLETE || PRSEV_u32TickFrNewState(pEv) > 100) {
			// タイムアウトは100msだがブロードキャスト送信ではタイムアウトする事はないはず。
			ToCoNet_Event_SetState(pEv, E_STATE_APP_REMOTE_FINISH);
		}
		break;

	case E_STATE_APP_REMOTE_FINISH:
		// 長押し検知時のスリープ (完了後短いスリープに入る)
		if (eEvent == E_EVENT_NEW_STATE) {
			if (!(sAppData.sIOData_now.u32BtmBitmap & sAppData.u32BtnMask_Special)) {
				if (u16OnPrsTx_Count) {
					u16OnPrsTx_Count--;
				}
			}

			if (u16OnPrsTx_Count == 0) {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEPING); // これでおしまいなので、通常の長いスリープに入る
			} else {
				ToCoNet_Event_SetState(pEv, E_STATE_APP_REMOTE_SMALL_SLEEP); // 短いスリープに入る (長押し検知のため)
			}
		}
		break;

	case E_STATE_APP_REMOTE_SMALL_SLEEP:
		if (eEvent == E_EVENT_NEW_STATE) {
			vSleep(sAppData.sFlash.sData.u16HoldDur_ms, FALSE, FALSE, FALSE);
		}
		break;

	case E_STATE_WAIT_TX:
		// 通常モードの送信待ち(完了後速やかにスリープに遷移する)
		if (eEvent == E_EVENT_APP_TX_COMPLETE || PRSEV_u32TickFrNewState(pEv) > 100) {
			// タイムアウトは100msだがブロードキャスト送信ではタイムアウトする事はないはず。

			if (IS_APPCONF_OPT_ACK_MODE() && u32evarg && sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
				// MODE7 & ACK モード時の IO 制御。Ack 受信が成功すれば PORT_BAUD をLOに設定する
				// その後、所定期間スリープした後、Hi に戻す
				vPortSetLo(PORT_BAUD);
			}

			ToCoNet_Event_SetState(pEv, E_STATE_FINISHED);
		}
		break;

	case E_STATE_FINISHED:
		if (eEvent == E_EVENT_NEW_STATE) {
			// チャタリングが落ち着くまで待つ（スリープ直後に再起床すると面倒なので）
			bChatteringSleep = TRUE;

			vSleep(100, FALSE, FALSE, TRUE);
		} else {
			// 定期スリープを実行する
			//vfPrintf(&sSerStream, "!INF SLEEP %dms."LB, sAppData.u32SleepDur);
			//SERIAL_vFlush(sSerStream.u8Device);
			//vfPrintf(&sSerStream, "!S"LB);

			ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEPING);
		}
		break;

	case E_STATE_APP_SLEEPING:
		if (eEvent == E_EVENT_NEW_STATE) {
			vSleep(sAppData.u32SleepDur, TRUE, FALSE, FALSE);
		}
		break;

	default:
		break;
	}
}


/** @ingroup MASTER
 * 電源投入時・リセット時に最初に実行される処理。本関数は２回呼び出される。初回は u32AHI_Init()前、
 * ２回目は AHI 初期化後である。
 *
 * - 各種初期化
 * - ToCoNet ネットワーク設定
 * - 設定IO読み取り
 * - 緊急フラッシュ消去処理
 * - 設定値の計算
 * - ハードウェア初期化
 * - イベントマシンの登録
 * - 本関数終了後は登録したイベントマシン、および cbToCoNet_vMain() など各種コールバック関数が
 *   呼び出される。
 *
 * @param bStart TRUE:u32AHI_Init() 前の呼び出し FALSE: 後
 */
void cbAppColdStart(bool_t bStart) {
	if (!bStart) {
		// before AHI initialization (very first of code)

		// Module Registration
		ToCoNet_REG_MOD_ALL();
	} else {
		// メモリのクリア
		memset(&sAppData, 0x00, sizeof(sAppData));
		memset(&(sAppData.sIOData_now), 0xFF, sizeof(tsIOData));
		memset(&(sAppData.sIOData_reserve), 0xFF, sizeof(tsIOData));
		vConfig_UnSetAll(&sAppData.sConfig_UnSaved);

		// デフォルトのネットワーク指定値
		sToCoNet_AppContext.u8TxMacRetry = 3; // MAC再送回数（JN516x では変更できない）
		sToCoNet_AppContext.u32AppId = APP_ID; // アプリケーションID
		sToCoNet_AppContext.u32ChMask = CHMASK; // 利用するチャネル群（最大３つまで）
		sToCoNet_AppContext.u8Channel = CHANNEL; // デフォルトのチャネル

		sToCoNet_AppContext.u8CCA_Level = 1; // CCA は最小レベルで設定 (Level=1, Retry=0 が最小）
		sToCoNet_AppContext.u8CCA_Retry = 1; // 再試行は１回のみ

		//sToCoNet_AppContext.u8CPUClk = 3;
		//sToCoNet_AppContext.u8TxMacRetry = 1; // Tx の再送回数

		sAppData.u8MaxIoCount = MAX_IO;
		u32_PORT_INPUT_MASK = PORT_INPUT_MASK;
#ifdef JN514x
		// STRONG の場合最後のDIO3を未使用にする
		if(ToCoNet_bIsStrong()) {
			sAppData.u8MaxIoCount--; // 末尾の DIO3 を削除
			u32_PORT_INPUT_MASK &= ~((1UL << 2) | (1UL << 3)); // DIO2/3 を除外
		}
#endif

		// フラッシュの読み出し
		sAppData.bFlashLoaded = bFlash_Read(&sAppData.sFlash, FLASH_SECTOR_NUMBER - 1, 0);

		// Version String のチェック
		if (sAppData.bFlashLoaded &&
			(   (APP_ID != sAppData.sFlash.sData.u32appkey)
			 || (VERSION_U32 != sAppData.sFlash.sData.u32ver) )) {
			sAppData.bFlashLoaded = FALSE;
		}

		// フラッシュ設定値の反映
		if (sAppData.bFlashLoaded) {
			sToCoNet_AppContext.u32AppId = sAppData.sFlash.sData.u32appid;
			// sToCoNet_AppContext.u8Channel = sAppData.sFlash.sData.u8ch; // チャネルマネージャで決定するので設定不要
			sToCoNet_AppContext.u32ChMask = sAppData.sFlash.sData.u32chmask;
			sToCoNet_AppContext.u8TxPower = sAppData.sFlash.sData.u8pow; // 出力の設定

			if (sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_MAC_NODE) {
				sAppData.eNwkMode = E_NWKMODE_MAC_DIRECT;
			} else
			if (sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_SILENT) {
				sAppData.eNwkMode = E_NWKMODE_MAC_DIRECT;
			} else {
				sAppData.bFlashLoaded = 0;
			}

			//sToCoNet_AppContext.u16TickHz = 1000; // App_IO では常に 1000Hz 稼働とする
			if (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT) {
				sToCoNet_AppContext.u16TickHz = 1000; // 低レイテンシモードでは 1KHz 動作
			}
		}

		// フラッシュのロードに失敗したとき
		if (sAppData.bFlashLoaded != TRUE) {
			// 構造体にデフォルト値を格納する
			vConfig_SetDefaults(&(sAppData.sFlash.sData));
		}
		// ヘッダの１バイト識別子を AppID から計算
		sAppData.u8AppIdentifier = u8CCITT8((uint8*)&sToCoNet_AppContext.u32AppId, 4); // APP ID の CRC8

		// IOより状態を読み取る (ID など)
		sAppData.u32DIO_startup = ~u32PortReadBitmap(); // この時点では全部入力ポート

		// version info
		sAppData.u32ToCoNetVersion = ToCoNet_u32GetVersion();

		// ToCoNet の制御 Tick [ms]
		sAppData.u16ToCoNetTickDelta_ms = 1000 / sToCoNet_AppContext.u16TickHz;

		// その他ハードウェアの初期化
		vInitHardware(FALSE);

		// 論理IDの設定チェック、その他設定値のチェック
		//  IO の設定を優先し、フラッシュ設定で矛盾するものについてはデフォルト値を書き直す。
		if (IS_LOGICAL_ID_CHILD(au8IoModeTbl_To_LogicalID[sAppData.u8Mode])) {
			// 子機IDはフラッシュ値が設定されていれば、これを採用
			if (sAppData.bFlashLoaded) {
				sAppData.u8AppLogicalId = sAppData.sFlash.sData.u8id;
			}

			if (!IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId )) {
				sAppData.u8AppLogicalId = au8IoModeTbl_To_LogicalID[sAppData.u8Mode];
			}
		}

		// 論理IDを121,122に保存した場合、親機で起動する
		if (sAppData.bFlashLoaded) {
			if (sAppData.sFlash.sData.u8id == 121) {
				sAppData.u8Mode = 1; // 親機のモード番号
				sAppData.u8AppLogicalId = au8IoModeTbl_To_LogicalID[sAppData.u8Mode]; // 論理IDを設定
			} else
			if (sAppData.sFlash.sData.u8id == 122) {
				sAppData.u8Mode = 2; // 親機のモード番号
				sAppData.u8AppLogicalId = au8IoModeTbl_To_LogicalID[sAppData.u8Mode]; // 論理IDを設定
			}
		}

#if 0
#warning "DEBUG"
		if (sAppData.u8Mode == E_IO_MODE_CHILD_SLP_1SEC) {
			sAppData.u8Mode = E_IO_MODE_CHILD_SLP_10SEC;
		}
#endif

		// 各モード依存の初期値の設定など
		switch(sAppData.u8Mode) {
		case E_IO_MODE_PARNET:
			sAppData.u8AppLogicalId = LOGICAL_ID_PARENT;
			break;

		case E_IO_MODE_ROUTER:
			sAppData.u8AppLogicalId = LOGICAL_ID_REPEATER;
			break;

		case E_IO_MODE_CHILD_SLP_1SEC:
			if (!sAppData.u32SleepDur) {
				if (sAppData.bFlashLoaded) {
					sAppData.u32SleepDur = sAppData.sFlash.sData.u16SleepDur_ms;
				} else {
					sAppData.u32SleepDur = MODE4_SLEEP_DUR_ms;
				}
			}
			break;

		case E_IO_MODE_CHILD_SLP_10SEC:
			if (!sAppData.u32SleepDur) {
				if (sAppData.bFlashLoaded) {
					sAppData.u32SleepDur = sAppData.sFlash.sData.u16SleepDur_s * 1000L;
				} else {
					sAppData.u32SleepDur = MODE7_SLEEP_DUR_ms;
				}
			}
			break;

		case E_IO_MODE_CHILD_CONT_TX:
			break;

		case E_IO_MODE_CHILD:
			break;

		default: // 未定義機能なので、SILENT モードにする。
			sAppData.u8AppLogicalId = 255;
			sAppData.sFlash.sData.u8role = E_APPCONF_ROLE_SILENT;
			break;
		}

		// FPS のビットマスク
		sAppData.u8FpsBitMask = 1;
		if (sAppData.bFlashLoaded) {
			// 4fps: 1111
			// 8fps:  111 (64/8 -1)
			// 16pfs:  11 (64/16-1)
			// 32fps:   1 (64/32-1)
			sAppData.u8FpsBitMask = 64 / sAppData.sFlash.sData.u8Fps - 1;
			// DBGOUT(0, "fps mask = %x"LB, sAppData.u8FpsBitMask);
		}

		// IO設定に基づきチャネルを設定する
		vChangeChannelPreset(sAppData.u8ChCfg);

		// 親機子機が決まったので IO 設定を行う
		vInitHardware_IOs(TRUE);

		// ショートアドレスの設定(決めうち)
		sToCoNet_AppContext.u16ShortAddress = SERCMD_ADDR_CONV_TO_SHORT_ADDR(sAppData.u8AppLogicalId);

		// UART の初期化
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

		// その他の初期化
		DUPCHK_vInit(&sDupChk_IoData); // 重複チェック用
		DUPCHK_vInit(&sDupChk_SerMsg); // 重複チェック用

		if (!(IS_APPCONF_ROLE_SILENT_MODE())) {
			// 状態遷移マシン、RxOnIdle の設定
			switch(sAppData.u8Mode) {
			case E_IO_MODE_PARNET:
			case E_IO_MODE_ROUTER:
				ToCoNet_Event_Register_State_Machine(vProcessEvCorePwr); // 常時通電用の処理
				sAppData.prPrsEv = (void*)vProcessEvCorePwr;
				// App_IO では親機と中継機のみ受信できる
				sToCoNet_AppContext.bRxOnIdle = TRUE;
				break;

			case E_IO_MODE_CHILD:
			case E_IO_MODE_CHILD_CONT_TX:
				ToCoNet_Event_Register_State_Machine(vProcessEvCorePwr); // 常時通電用の処理
				sAppData.prPrsEv = (void*)vProcessEvCorePwr;

				if (IS_APPCONF_OPT_CHILD_RECV_OTHER_NODES()) {
					// 子機を受信可能モードに設定する
					sToCoNet_AppContext.bRxOnIdle = TRUE;
				} else {
					sToCoNet_AppContext.bRxOnIdle = FALSE;
				}
				break;

			case E_IO_MODE_CHILD_SLP_1SEC:
			case E_IO_MODE_CHILD_SLP_10SEC:
				ToCoNet_Event_Register_State_Machine(vProcessEvCoreSlp); // スリープ用の処理
				sAppData.prPrsEv = (void*)vProcessEvCoreSlp;
				sToCoNet_AppContext.bRxOnIdle = FALSE;
				break;
			default: // 未定義機能なので、SILENT モードにする。
				break;
			}

			// MAC の初期化
			ToCoNet_vMacStart();

			// 主状態遷移マシンの登録
			ToCoNet_Event_Register_State_Machine(vProcessEvCore);
		}
	}
}

/** @ingroup MASTER
 * スリープ復帰後に呼び出される関数。\n
 * 本関数も cbAppColdStart() と同様に２回呼び出され、u32AHI_Init() 前の
 * 初回呼び出しに於いて、スリープ復帰要因を判定している。u32AHI_Init() 関数は
 * これらのレジスタを初期化してしまう。
 *
 * - 変数の初期化（必要なもののみ）
 * - ハードウェアの初期化（スリープ後は基本的に再初期化が必要）
 * - イベントマシンは登録済み。
 *
 * @param bStart TRUE:u32AHI_Init() 前の呼び出し FALSE: 後
 */
void cbAppWarmStart(bool_t bStart) {
	if (!bStart) {
		// before AHI init, very first of code.
		//  to check interrupt source, etc.

		sAppData.bWakeupByButton = FALSE;
		u32WakeBtnStatus = u32AHI_DioWakeStatus();

		if(u8AHI_WakeTimerFiredStatus()) {
			;
		} else
		if(u32WakeBtnStatus & u32_PORT_INPUT_MASK) {
			// woke up from DIO events
			sAppData.bWakeupByButton = TRUE;
		}
	} else {
		// データ領域の初期化
		memset(&(sAppData.sIOData_now), 0xFF, sizeof(tsIOData));

		// いくつかのデータは復元
		sAppData.sIOData_now.u32BtmUsed = sAppData.sIOData_reserve.u32BtmUsed;

		// 変数の初期化（必要なものだけ）
		sAppData.u16CtTimer0 = 0; // このカウンタは、起動時からのカウントとする
		sAppData.u8IOFixState = E_IO_FIX_STATE_NOT_READY; // IO読み取り状態の確定待ちフラグ

		// スリープ条件の判定
		// チャタリング抑制期間からのスリープ復帰
		if (bChatteringSleep) {
			bChatteringSleep = FALSE;
			if (!u16OnPrsTx_Count) {
				// IO 状態を基に戻す
				if (IS_APPCONF_OPT_ACK_MODE() && sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
					vPortSetHi(PORT_BAUD);
				}

				// abAppWarmStart を抜けると MAC 層の初期化など大きな処理が続くため、ここでスリープ
				vSleep(sAppData.u32SleepDur, FALSE, FALSE, FALSE);
			}
		}

		// その他ハードウェアの初期化（基本、起動時に毎回実行する）
		vInitHardware(TRUE);
		vInitHardware_IOs(TRUE);

		// UART の初期化
		ToCoNet_vDebugInit(&sSerStream);
		ToCoNet_vDebugLevel(0);

		// その他の初期化
		DUPCHK_vInit(&sDupChk_IoData);
		DUPCHK_vInit(&sDupChk_SerMsg);

		// MAC の開始
		ToCoNet_vMacStart();
	}
}

/** @ingroup MASTER
 * 本関数は ToCoNet のメインループ内で必ず１回は呼び出される。
 * ToCoNet のメインループでは、CPU DOZE 命令を発行しているため、割り込みなどが発生した時に
 * 呼び出されるが、処理が無い時には呼び出されない。
 * しかし TICK TIMER の割り込みは定期的に発生しているため、定期処理としても使用可能である。
 *
 * - シリアルの入力チェック
 */
void cbToCoNet_vMain(void) {
	vHandleSerialInput(); // シリアルポートの処理
}

/** @ingroup MASTER
 * パケットの受信完了時に呼び出されるコールバック関数。\n
 * パケットの種別によって具体的な処理関数にディスパッチしている。
 * データ種別は psRx->u8Cmd (ToCoNet のパケットヘッダに含まれます) により識別される。
 *
 * - パケット種別
 *   - TOCONET_PACKET_CMD_APP_DATA : シリアル電文パケット
 *   - TOCONET_PACKET_CMD_APP_USER_IO_DATA : IO状態の伝送
 *   - TOCONET_PACKET_CMD_APP_USER_IO_DATA_EXT : シリアル電文による IO 状態の伝送
 *
 * @param psRx 受信パケット
 */
void cbToCoNet_vRxEvent(tsRxDataApp *psRx) {
	//uint8 *p = pRx->auData;

	DBGOUT(3, "Rx packet (cm:%02x, fr:%08x, to:%08x)"LB, psRx->u8Cmd, psRx->u32SrcAddr, psRx->u32DstAddr);

	if (IS_APPCONF_ROLE_SILENT_MODE()
		|| sAppData.u8Mode == E_IO_MODE_CHILD_SLP_1SEC
		|| sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
		// SILENT, 1秒スリープ, 10秒スリープでは受信処理はしない。
		return;
	}

	// 暗号化モードで平文は無視する
	if (IS_CRYPT_MODE()) {
		if (!psRx->bSecurePkt) {
			DBGOUT(5, LB"Recv Plain Pkt!");
			return;
		}
	}

	switch (psRx->u8Cmd) {
	case TOCONET_PACKET_CMD_APP_USER_IO_DATA: // IO状態の伝送
		vReceiveIoData(psRx);
		break;
	case TOCONET_PACKET_CMD_APP_USER_IO_DATA_EXT: // IO状態の伝送(UART経由)
		vReceiveIoSettingRequest(psRx);
		break;
	case TOCONET_PACKET_CMD_APP_USER_SERIAL_MSG: // IO状態の伝送(UART経由)
		vReceiveSerialMsg(psRx);
		break;
	}
}


/** @ingroup MASTER
 * 送信完了時に呼び出されるコールバック関数。
 *
 * - IO 送信完了イベントはイベントマシンにE_EVENT_APP_TX_COMPLETEイベントを伝達する。
 * - シリアルメッセージの一連のパケット群の送信完了も検出しsSerSeqTx.bWaitComplete
 *   フラグをリセットする。
 *
 * @param u8CbId 送信時に設定したコールバックID
 * @param bStatus 送信ステータス
 */
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
	//uint8 *q = au8SerOutBuff;
	if (IS_APPCONF_ROLE_SILENT_MODE()) {
		return;
	}

	//DBGOUT(0, "<Tx %d=%d>", u8CbId, bStatus);

	// IO 関連の送信が完了した
	if (sAppData.sIOData_now.i16TxCbId >= 0
		&& u8CbId == sAppData.sIOData_now.i16TxCbId) {
		// スリープを行う場合は、このイベントを持ってスリープ遷移
		ToCoNet_Event_Process(E_EVENT_APP_TX_COMPLETE, bStatus, sAppData.prPrsEv);
	}

	return;
}

/** @ingroup MASTER
 * ネットワーク層などのイベントが通達される。\n
 * 本アプリケーションでは特別な処理は行っていない。
 *
 * @param ev
 * @param u32evarg
 */
void cbToCoNet_vNwkEvent(teEvent ev, uint32 u32evarg) {
	if (IS_APPCONF_ROLE_SILENT_MODE()) {
		return;
	}

	switch(ev) {
	case E_EVENT_TOCONET_NWK_START:
		break;

	case E_EVENT_TOCONET_NWK_DISCONNECT:
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * ハードウェア割り込み時に呼び出される。本処理は割り込みハンドラではなく、割り込みハンドラに登録された遅延実行部による処理で、長い処理が記述可能である。
 * 本アプリケーションに於いては、ADC/DIの入力状態のチェック、64fps のタイマーイベントの処理などを行っている。
 *
 * - E_AHI_DEVICE_SYSCTRL
 *   - DI割り込みの処理を行う。これは、低レイテンシモードでの処理である。
 *
 * - E_AHI_DEVICE_TICK_TIMER : このイベントは ToCoNet 組み込みで、ToCoNet の制御周期 (sToCoNet_AppContext.u16TickHz) を
 *   実現するためのタイマーです。ユーザが TickTimer の制御を変更したりすると ToCoNet は動作しなくなります。
 *
 *   - Di入力の変化の確認。変化が有れば、sAppData.sIOData_now 構造体に結果を格納する。
 *     低レイテンシモードの時は、この判定を起点として送信を行う。
 *
 * - E_AHI_DEVICE_TIMER0 : TICK_TIMER から分周して制御周期を作っても良いのですが、TIMER_0 を使用しています。
 *   - カウンタのインクリメント処理
 *   - ADC の完了確認
 *   - パケット重複チェックアルゴリズムのタイムアウト処理
 *   - DIのカウンタ処理 (インタラクティブモードでカウンタ終了時にもパケットを送信する処理を行う）
 *   - イベントマシンに TIMER0 イベントを発行
 *   - インタラクティブモード時の画面再描画
 *
 * @param u32DeviceId
 * @param u32ItemBitmap
 */
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	switch (u32DeviceId) {
	case E_AHI_DEVICE_SYSCTRL:
		// 低レイテンシモードでは割り込みが入る。
		// 割り込みハンドラを抜けた後のアプリケーション処理がこの部分。
		if (sAppData.u8IOFixState == E_IO_FIX_STATE_READY
				&& sAppData.u8Mode != E_IO_MODE_CHILD_SLP_1SEC   // SLEEP モードでない
				&& sAppData.u8Mode != E_IO_MODE_CHILD_SLP_10SEC  // SLEEP モードでない
		) {
			int i;
			bool_t bTransmit = FALSE;

			/* DIの入力ピンの番号を調べる。
			 *
			 *  ボタンを猿みたいに押してみたが DIO の割り込みは同時に２ビット報告されることは無く、
			 *  順 次処理されるようだ。しかしながら、同時処理されても良いようなコードにしておく。
			 */
			DBGOUT(1, LB"BTN: ");
			for (i = 0; i < sAppData.u8MaxIoCount; i++) {
				/* DI検知したポートについては sAppData.sIOData_now.au8Input[] に値を設定する。
				 *
				 * この値の下位１ビットが、１になると Lo 判定したことを意味する。
				 * この値の上位４ビットは、再度の割り込みを検知しても無視するためのカウンタとして
				 * 用いる。このカウンタは E_AHI_DEVICE_TIMER0 イベントで処理する。
				 */
				DBGOUT(1, "%c", u32ItemBitmap & (1UL << au8PortTbl_DIn[i]) ? '1' : '0');

				if (u32ItemBitmap & (1UL << au8PortTbl_DIn[i])) { // 押し下げを検知したポート
					uint8 u8stat = sAppData.sIOData_now.au8Input[i]; // 元の値を取り出す。
					uint8 u8ct = (u8stat & 0xF0) >> 4; // 上位４ビットは、前回の同様の割り込み受信からの 64fps カウンタ
					// カウンタ値が無い場合は、個の割り込みを有効とする。
					if (u8ct == 0) {
						sAppData.sIOData_now.au8Input[i] = (LOW_LATENCY_DELAYED_TRANSMIT_COUNTER * 0x10) + 1;
						bTransmit = TRUE;
					} else {
						// カウンタ値が有る場合は、直前に押されたためチャタリングなどが考えられ処理しない
						;
					}
				}
			}

			// いずれかのポートの割り込みが有効であった場合。
			if (bTransmit) {
				/* 速やかに送信する
				 *   ポートの変更対象フラグを、この割り込みで入力検知したものとする。
				 *   そうしないと、関係ないビットが変更されてしまう
				 */
				uint32 u32used = sAppData.sIOData_now.u32BtmUsed; // 関数呼び出し中だけ値を変更する
				sAppData.sIOData_now.u32BtmUsed = u32ItemBitmap & u32_PORT_INPUT_MASK; // 割り込みでLoになったDINだけ変更対照として送信する
				sAppData.sIOData_now.i16TxCbId = i16TransmitIoData(TRUE); // 送信処理を行う
				sAppData.sIOData_now.u32BtmUsed = u32used | (u32ItemBitmap & u32_PORT_INPUT_MASK); //値を復元する
			}
		}
		break;

	case E_AHI_DEVICE_ANALOGUE: //ADC完了時にこのイベントが発生する。
		break;

	case E_AHI_DEVICE_TICK_TIMER: //比較的頻繁な処理
		// ボタンの判定を行う。
		_C {
			uint32 bmPorts, bmChanged, i;

			if (bBTM_GetState(&bmPorts, &bmChanged)) {
				// 読み出し値を格納する
				for (i = 0; i < sAppData.u8MaxIoCount; i++) {
					uint8 u8stat = (sAppData.sIOData_now.au8Input[i] == 0xFF) ? 0 : sAppData.sIOData_now.au8Input[i]; // TODO
					// LSBを設定する
					if (bmPorts & (1UL << au8PortTbl_DIn[i])) {
						u8stat |= 0x01;
					} else {
						u8stat &= 0xFE;
					}
					sAppData.sIOData_now.au8Input[i] = u8stat;
				}
				sAppData.sIOData_now.u32BtmBitmap = bmPorts; // au8Input と冗長だが両方管理する。

				// EI関連のポートが変化した
				if (bmChanged & ((1UL << PORT_EI1) | (1UL << PORT_EI2))) {
					// チャネルを変更する
					sAppData.u8ChCfg =
							  (bmPorts & (1UL << PORT_EI1) ? 1 : 0)
							+ (bmPorts & (1UL << PORT_EI2) ? 2 : 0);
					vChangeChannelPreset(sAppData.u8ChCfg);
					// この２ビットをクリアしておく
					bmChanged &= ~((1UL << PORT_EI1) | (1UL << PORT_EI2));
				}

				if (bmChanged) { // 入力ポートの値が確定したか、変化があった
					// 利用入力ポートの判定
					if (sAppData.sIOData_now.u32BtmUsed == 0xFFFFFFFF) {
						sAppData.sIOData_now.u32BtmUsed = bmPorts; // 一番最初の確定時に Lo のポートは利用ポート
					} else {
						sAppData.sIOData_now.u32BtmUsed |= bmPorts; // 利用ポートのビットマップは一度でもLo確定したポート
					}

					// 変化ポートの判定
					if (sAppData.u8IOFixState == E_IO_FIX_STATE_READY) {
						// 初回確定後
						sAppData.sIOData_now.u32BtmChanged |= bmChanged; // 変化フラグはアプリケーションタスクに変更させる
					} else {
						// 初回確定時(スリープ復帰後も含む)
						sAppData.sIOData_now.u32BtmChanged = bmChanged; // 初回は変化を報告
					}

					// IO確定ビットを立てる
					sAppData.u8IOFixState |= E_IO_FIX_STATE_READY;
				}
			}

			// 低レイテンシモードならここで送信を行う。
			static bool_t bLowLatencyTxCond = 0xFF; // 判定回数を減らすため、このフラグを利用する
			if (bLowLatencyTxCond == 0xFF) {
				if (sAppData.u8IOFixState == E_IO_FIX_STATE_READY) {
					if ((sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT)
						&& (sAppData.u8Mode != E_IO_MODE_CHILD_SLP_10SEC)
						&& (sAppData.u8Mode != E_IO_MODE_CHILD_SLP_1SEC)
						&& !IS_APPCONF_OPT_ON_PRESS_TRANSMIT() // リモコンモード＆低レイテンシの場合はここでは送信しない
					) {
						bLowLatencyTxCond = TRUE;
					} else {
						bLowLatencyTxCond = FALSE;
					}
				}
			}
			if (bLowLatencyTxCond == TRUE && sAppData.sIOData_now.u32BtmChanged) {
				sAppData.sIOData_now.i16TxCbId = i16TransmitIoData(TRUE);
				sAppData.sIOData_now.u32BtmChanged = 0;
			}
		}
		break;

	case E_AHI_DEVICE_TIMER0:
		// タイマーカウンタをインクリメントする (64fps なので 64カウントごとに１秒)
		sAppData.u32CtTimer0++;
		sAppData.u16CtTimer0++;

		// 重複チェックのタイムアウト処理
		if ((sAppData.u32CtTimer0 & 0xF) == 0) {
			DUPCHK_bFind(&sDupChk_IoData, 0, NULL);
			DUPCHK_bFind(&sDupChk_SerMsg, 0, NULL);
		}

		/* インタラクティブモードのカウンタ処理。
		 * カウンタが0になったときに送信フラグを立てる。
		 * 1.3.4 カウンタが0までは押し下げフラグを維持
		 */
		{
			int i;

			// Input ビットのカウンタビットをカウントダウンする。
			bool_t bUpdated = FALSE;
			for(i = 0; i < sAppData.u8MaxIoCount; i++) {
				uint8 u8stat = sAppData.sIOData_now.au8Input[i];
				if (u8stat == 0xFF) continue; // 初期化直後（まだ値が入っていない）

				uint8 u8ct = u8stat >> 4;

				if (u8ct) {
					u8ct--;
					if (u8ct == 0) {
						// 送信要求
						bUpdated = TRUE;
					}
				}

				u8stat = (u8ct << 4) + (u8stat & 0x0F);
				sAppData.sIOData_now.au8Input[i] = u8stat;
			}
			if (bUpdated) {
				sAppData.sIOData_now.u32BtmChanged |= 0x80000000;
			}
		}

		// イベント処理部分にイベントを送信
		if (sAppData.prPrsEv && (sAppData.u32CtTimer0 & 1)) {
			ToCoNet_Event_Process(E_EVENT_APP_TICK_A, 0, sAppData.prPrsEv);
		}

		// シリアル画面制御のためのカウンタ
		if (!(--u16HoldUpdateScreen)) {
			if (sSerCmdIn.bverbose) {
				vSerUpdateScreen();
			}
		}
		break;

	default:
		break;
	}
}

/** @ingroup MASTER
 * 割り込みハンドラ。ここでは長い処理は記述してはいけない。
 *
 * - TIMER_0\n
 *   - JN514x での DAC 出力
 * - TICK_TIMER\n
 *   - ADCの実行管理
 *   - ボタン入力判定管理
 */
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
	uint8 u8handled = FALSE;

	switch (u32DeviceId) {
	case E_AHI_DEVICE_TIMER0:
		break;

	case E_AHI_DEVICE_TICK_TIMER:
		// ボタンハンドラの駆動
		if (sAppData.pr_BTM_handler) {
			// ハンドラを稼働させる
			(*sAppData.pr_BTM_handler)(sAppData.u16ToCoNetTickDelta_ms);
		}

		// ホールドモードの制御
		if (sAppData.sFlash.sData.u32HoldMask) {
			int i = 0, j = 1;
			for (; i < sAppData.u8MaxIoCount; i++, j <<= 1) {
				if (sAppData.au16HoldBtn[i]) {
					// カウントダウンしていく
					if (sAppData.au16HoldBtn[i] < sAppData.u16ToCoNetTickDelta_ms) {
						sAppData.au16HoldBtn[i] = 0;
					} else {
						sAppData.au16HoldBtn[i] -= sAppData.u16ToCoNetTickDelta_ms;
					}

					// カウント値が 0 になったら IO を Hi に戻す
					if (sAppData.au16HoldBtn[i] == 0) {
						vPortSetHi(au8PortTbl_DOut[i]);
					}
				}
			}
		}
		break;

	default:
		break;
	}

	return u8handled;
}

/** @ingroup MASTER
 * ハードウェアの初期化を行う。スリープ復帰時も原則同じ初期化手続きを行う。
 *
 * - 管理構造体のメモリ領域の初期化
 * - DO出力設定
 * - DI入力設定
 * - DI割り込み設定 (低レイテンシモード)
 * - M1-M3 の読み込み
 * - UARTの初期化
 * - ADC3/4 のプルアップ停止
 * - タイマー用の未使用ポートを汎用IOに解放する宣言
 * - 秒64回のTIMER0の初期化と稼働
 * - ADC/DAC(JN514x)/PWM の初期化
 * - I2Cの初期化
 *
 * @param f_warm_start TRUE:スリープ復帰時
 */
static void vInitHardware(int f_warm_start) {
	// メモリのクリア
	memset(&sTimerApp, 0, sizeof(tsTimerContext));

	// モード設定
	vPortAsInput(PORT_CONF1);
	vPortAsInput(PORT_CONF2);
	vPortAsInput(PORT_CONF3);
	sAppData.u8Mode = (bPortRead(PORT_CONF1) | (bPortRead(PORT_CONF2) << 1) | (bPortRead(PORT_CONF3) << 2));

	// チャネル設定 (0 ならデフォルトorフラッシュ値、1, 2, 3 は設定値)
	vPortAsInput(PORT_EI1);
	vPortAsInput(PORT_EI2);
	sAppData.u8ChCfg =
			  (bPortRead(PORT_EI1) ? 1 : 0)
			+ (bPortRead(PORT_EI2) ? 2 : 0);

	// UART 設定
	{
		// ACK モードでは BAUD ピンを出力用に使用する
		uint32 u32baud;
		bool_t bPortBPS;

		if (IS_APPCONF_OPT_ACK_MODE() && sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
			// ACK_MODE かつ 10秒SLEEP モード時は、BPSピンを出力として使用する
			bPortBPS = FALSE;
			u32baud = UART_BAUD;
		} else {
			vPortAsInput(PORT_BAUD);
			bPortBPS = bPortRead(PORT_BAUD);
			u32baud = bPortBPS ? UART_BAUD_SAFE : UART_BAUD;
		}

		tsUartOpt sUartOpt;
		memset(&sUartOpt, 0, sizeof(tsUartOpt));

		// BAUD ピンが GND になっている場合、かつフラッシュの設定が有効な場合は、設定値を採用する (v1.0.3)
		if (sAppData.bFlashLoaded && bPortBPS) {
			u32baud = sAppData.sFlash.sData.u32baud_safe;
			sUartOpt.bHwFlowEnabled = FALSE;
			sUartOpt.bParityEnabled = UART_PARITY_ENABLE;
			sUartOpt.u8ParityType = UART_PARITY_TYPE;
			sUartOpt.u8StopBit = UART_STOPBITS;

			// 設定されている場合は、設定値を採用する (v1.0.3)
			switch(sAppData.sFlash.sData.u8parity) {
			case 0:
				sUartOpt.bParityEnabled = FALSE;
				break;
			case 1:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_ODD_PARITY;
				break;
			case 2:
				sUartOpt.bParityEnabled = TRUE;
				sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
				break;
			}

			vSerialInit(u32baud, &sUartOpt);
		} else {
			vSerialInit(u32baud, NULL);
		}
	}

	// タイマの未使用ポートの解放（汎用ＩＯとして使用するため）
#if defined(JN516x)
	vAHI_TimerFineGrainDIOControl(0x7F); // bit 0,1,2 をセット (TIMER0 の各ピンを解放する, PWM1..4 は使用する)
#elif defined(JN514x)
	vAHI_TimerFineGrainDIOControl(0x7F); // PWM出力は無効に
#endif

	// 秒64回のTIMER0の初期化と稼働
	sTimerApp.u8Device = E_AHI_DEVICE_TIMER0;
	sTimerApp.u16Hz =64;
	sTimerApp.u8PreScale = 4; // 15625ct@2^4

	vTimerConfig(&sTimerApp);
	vTimerStart(&sTimerApp);
}


/** @ingroup MASTER
 * ハードウェアの初期化を行う。スリープ復帰時も原則同じ初期化手続きを行う。
 *
 * - 管理構造体のメモリ領域の初期化
 * - DO出力設定
 * - DI入力設定
 * - DI割り込み設定 (低レイテンシモード)
 * - M1-M3 の読み込み
 * - UARTの初期化
 * - ADC3/4 のプルアップ停止
 * - タイマー用の未使用ポートを汎用IOに解放する宣言
 * - 秒64回のTIMER0の初期化と稼働
 * - ADC/DAC(JN514x)/PWM の初期化
 * - I2Cの初期化
 *
 * @param f_warm_start TRUE:スリープ復帰時
 */
static void vInitHardware_IOs(int f_warm_start) {
	int i;

	if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
		// 子機は入力側とする

		// 入力の設定
		for (i = 0; i < sAppData.u8MaxIoCount; i++) {
			vPortAsInput(au8PortTbl_DIn[i]);
		}

		// 低レイテンシで入力を行う処理(SLEEPモード時は割り込みは無視)
		if ((sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT)
			&& sAppData.u8Mode != E_IO_MODE_CHILD_SLP_10SEC
			&& sAppData.u8Mode != E_IO_MODE_CHILD_SLP_1SEC
		) {
			// 割り込みを有効にする
			vAHI_DioInterruptEnable(u32_PORT_INPUT_MASK, 0); // 割り込みの登録
			vAHI_DioInterruptEdge(0, u32_PORT_INPUT_MASK); // 割り込みエッジの登録
		} else {
			vAHI_DioInterruptEnable(0, u32_PORT_INPUT_MASK); // 割り込みを無効化
		}

		// 送信完了状態の設定
		if (IS_APPCONF_OPT_ACK_MODE() && sAppData.u8Mode == E_IO_MODE_CHILD_SLP_10SEC) {
			// MODE7&ACKモード時には PORT_BAUD は出力ピンとして使用する
			vPortSetHi(PORT_BAUD);
			vPortAsOutput(PORT_BAUD);
		}

		// button Manager (for Input)
		sAppData.sBTM_Config.bmPortMask = u32_PORT_INPUT_MASK | (1UL << PORT_EI1) | (1UL << PORT_EI2); // ここでは EI1,2 の監視も行う

	} else if (IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
		// 親機は出力側とする

		// 出力の設定
		for (i = 0; i < sAppData.u8MaxIoCount; i++) {
			vPortAsOutput(au8PortTbl_DOut[i]);
			//vPortSetHi(au8PortTbl_DOut[i]);
			if (sAppData.sIOData_reserve.au8Output[i] == 0xFF) {
				// 始動時
				vPortSetHi(au8PortTbl_DOut[i]);
			} else {
				// スリープ回復後
				vPortSet_TrueAsLo(au8PortTbl_DOut[i], sAppData.sIOData_reserve.au8Output[i]);
			}
		}

		// button Manager (for Input)
		sAppData.sBTM_Config.bmPortMask = (1UL << PORT_EI1) | (1UL << PORT_EI2); // ここでは EI1,2 の監視も行う

	} else {
		// 親機子機以外、何もしない

		// button Manager (for Input)
		sAppData.sBTM_Config.bmPortMask = (1UL << PORT_EI1) | (1UL << PORT_EI2); // ここでは EI1,2 の監視も行う
	}

	// ボタン監視の有効化
	sAppData.sBTM_Config.u16Tick_ms = sAppData.u16ToCoNetTickDelta_ms; //低レイテンシでは1ms, デフォルトは4ms
	sAppData.sBTM_Config.u8MaxHistory = 5; //連続参照回数
	sAppData.sBTM_Config.u8DeviceTimer = 0xFF; // TickTimer を流用する。
	sAppData.pr_BTM_handler = prBTM_InitExternal(&sAppData.sBTM_Config);
	vBTM_Enable();

	// モード設定ピンで Lo になっているポートはプルアップ停止
	// Lo でない場合は、プルアップ停止をするとリーク電流が発生する
	// ※ 暗電流に神経質な mode4, 7 のみ設定する。
	// スリープ時は設定ピンを途中で変更しない前提。
	if (sAppData.u8Mode == 4) {
		vPortDisablePullup(PORT_CONF3);
	} else if (sAppData.u8Mode == 7) {
		vPortDisablePullup(PORT_CONF1);
		vPortDisablePullup(PORT_CONF2);
		vPortDisablePullup(PORT_CONF3);
	}
	if (sAppData.u8Mode == 4 || sAppData.u8Mode == 7) {
		if (sAppData.u8ChCfg & 1) {
			vPortDisablePullup(PORT_EI1);
		}
		if (sAppData.u8ChCfg & 2) {
			vPortDisablePullup(PORT_EI2);
		}
	}

	// u32HoldMask に対応するマスクを計算しておく
	{
		int i = 0, j = 1;

		sAppData.u32BtnMask_Special = 0;
		for (; i < sAppData.u8MaxIoCount; i++, j <<= 1) {
			if (sAppData.sFlash.sData.u32HoldMask & j)
			sAppData.u32BtnMask_Special |= (1UL << au8PortTbl_DIn[i]);
		}
	}
}

/** @ingroup MASTER
 * チャネル設定を行う。
 * - EI1,EI2 の設定に基づきチャネルを変更する
 * @param u8preset
 */
static void vChangeChannelPreset(uint8 u8preset) {
	if (u8preset == 0) {
		// デフォルト値またはフラッシュ格納値を採用する
		sToCoNet_AppContext.u32ChMask = sAppData.sFlash.sData.u32chmask;
	} else {
		sToCoNet_AppContext.u32ChMask = au32ChMask_Preset[u8preset];
	}

	// 設定を反映する
	ToCoNet_vRfConfig();
}

/** @ingroup MASTER
 * UART を初期化する
 * @param u32Baud ボーレート
 */
void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
	/* Create the debug port transmit and receive queues */
	static uint8 au8SerialTxBuffer[512];
	static uint8 au8SerialRxBuffer[512];

	/* Initialise the serial port to be used for debug output */
	sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
	sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
	sSerPort.u32BaudRate = u32Baud;
	sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
	sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
	sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
	sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
	sSerPort.u8SerialPort = UART_PORT_MASTER;
	sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
	SERIAL_vInitEx(&sSerPort, pUartOpt);

	/* prepare stream for vfPrintf */
	sSerStream.bPutChar = SERIAL_bTxChar;
	sSerStream.u8Device = UART_PORT_MASTER;

	/* other initialization */
	INPSTR_vInit(&sSerInpStr, &sSerStream);
	memset(&sSerCmdIn, 0x00, sizeof(sSerCmdIn));

	sSerCmdIn.au8data = au8SerInBuff;
	sSerCmdIn.u16maxlen = sizeof(au8SerInBuff);
}

/** @ingroup MASTER
 * 始動時メッセージの表示を行う。
 */
static void vSerInitMessage() {
	vfPrintf(&sSerStream, LB"!INF "APP_NAME" V%d-%02d-%d, SID=0x%08X, LID=0x%02x"LB,
			VERSION_MAIN, VERSION_SUB, VERSION_VAR, ToCoNet_u32GetSerial(), sAppData.u8AppLogicalId);
	if (sAppData.bFlashLoaded == 0) {
		vfPrintf(&sSerStream, "!INF Default config (no save info)..." LB);
	}
#if defined(JN514x)
	vfPrintf(&sSerStream, "!INF DIO --> %021b"LB, sAppData.u32DIO_startup);
#elif defined(JN516x)
	vfPrintf(&sSerStream, "!INF DIO --> %020b"LB, sAppData.u32DIO_startup);
#endif
	if (IS_APPCONF_ROLE_SILENT_MODE()) {
		vfPrintf(&sSerStream, "!ERR SILENT MODE" LB);
	}
}

/** @ingroup MASTER
 * インタラクティブモードの画面を再描画する。
 * - 本関数は TIMER_0 のイベント処理時に u16HoldUpdateScreen カウンタがデクリメントされ、
 *   これが0になった時に呼び出される。
 *
 * - 設定内容、設定値、未セーブマーク*を出力している。FL_... のマクロ記述を参照。
 *
 */
static void vSerUpdateScreen() {
	V_PRINT("%c[2J%c[H", 27, 27); // CLEAR SCREEN
	V_PRINT("--- CONFIG/"APP_NAME" V%d-%02d-%d/SID=0x%08x/LID=0x%02x ---"LB,
			VERSION_MAIN, VERSION_SUB, VERSION_VAR, ToCoNet_u32GetSerial(), sAppData.u8AppLogicalId);

	// Application ID
	V_PRINT(" a: set Application ID (0x%08x)%c" LB,
			FL_IS_MODIFIED_u32(appid) ? FL_UNSAVE_u32(appid) : FL_MASTER_u32(appid),
			FL_IS_MODIFIED_u32(appid) ? '*' : ' ');

	// Device ID
	{
		uint8 u8DevID = FL_IS_MODIFIED_u8(id) ? FL_UNSAVE_u8(id) : FL_MASTER_u8(id);

		if (u8DevID == 0x00) { // unset
			V_PRINT(" i: set Device ID (--)%c"LB,
					FL_IS_MODIFIED_u8(id) ? '*' : ' '
					);
		} else {
			V_PRINT(" i: set Device ID (%d=0x%02x)%c"LB,
					u8DevID, u8DevID,
					FL_IS_MODIFIED_u8(id) ? '*' : ' '
					);
		}
	}

	V_PRINT(" c: set Channels (");
	{
		// find channels in ch_mask
		uint8 au8ch[MAX_CHANNELS], u8ch_idx = 0;
		int i;
		memset(au8ch,0,MAX_CHANNELS);
		uint32 u32mask = FL_IS_MODIFIED_u32(chmask) ? FL_UNSAVE_u32(chmask) : FL_MASTER_u32(chmask);
		for (i = 11; i <= 26; i++) {
			if (u32mask & (1UL << i)) {
				if (u8ch_idx) {
					V_PUTCHAR(',');
				}
				V_PRINT("%d", i);
				au8ch[u8ch_idx++] = i;
			}

			if (u8ch_idx == MAX_CHANNELS) {
				break;
			}
		}
	}
	V_PRINT(")%c" LB,
			FL_IS_MODIFIED_u32(chmask) ? '*' : ' ');

	V_PRINT(" x: set Tx Power (%d)%c" LB,
			FL_IS_MODIFIED_u8(pow) ? FL_UNSAVE_u8(pow) : FL_MASTER_u8(pow),
			FL_IS_MODIFIED_u8(pow) ? '*' : ' ');

	V_PRINT(" t: set mode4 sleep dur (%dms)%c" LB,
			FL_IS_MODIFIED_u16(SleepDur_ms) ? FL_UNSAVE_u16(SleepDur_ms) : FL_MASTER_u16(SleepDur_ms),
			FL_IS_MODIFIED_u16(SleepDur_ms) ? '*' : ' ');

	V_PRINT(" y: set mode7 sleep dur (%ds)%c" LB,
			FL_IS_MODIFIED_u16(SleepDur_s) ? FL_UNSAVE_u16(SleepDur_s) : FL_MASTER_u16(SleepDur_s),
			FL_IS_MODIFIED_u16(SleepDur_s) ? '*' : ' ');

	V_PRINT(" f: set mode3 fps (%d)%c" LB,
			FL_IS_MODIFIED_u8(Fps) ? FL_UNSAVE_u8(Fps) : FL_MASTER_u8(Fps),
			FL_IS_MODIFIED_u8(Fps) ? '*' : ' ');

	V_PRINT(" d: set hold mask (%012b)%c" LB,
			FL_IS_MODIFIED_u32(HoldMask) ? FL_UNSAVE_u32(HoldMask) : FL_MASTER_u32(HoldMask),
			FL_IS_MODIFIED_u32(HoldMask) ? '*' : ' ');

	V_PRINT(" D: set hold dur (%dms)%c" LB,
			FL_IS_MODIFIED_u16(HoldDur_ms) ? FL_UNSAVE_u16(HoldDur_ms) : FL_MASTER_u16(HoldDur_ms),
			FL_IS_MODIFIED_u16(HoldDur_ms) ? '*' : ' ');

	V_PRINT(" o: set Option Bits (0x%08X)%c" LB,
			FL_IS_MODIFIED_u32(Opt) ? FL_UNSAVE_u32(Opt) : FL_MASTER_u32(Opt),
			FL_IS_MODIFIED_u32(Opt) ? '*' : ' ');

	{
		uint32 u32baud = FL_IS_MODIFIED_u32(baud_safe) ? FL_UNSAVE_u32(baud_safe) : FL_MASTER_u32(baud_safe);
		if (u32baud & 0x80000000) {
			V_PRINT(" b: set UART baud (%x)%c" LB, u32baud,
					FL_IS_MODIFIED_u32(baud_safe) ? '*' : ' ');
		} else {
			V_PRINT(" b: set UART baud (%d)%c" LB, u32baud,
					FL_IS_MODIFIED_u32(baud_safe) ? '*' : ' ');
		}
	}

	{
		const uint8 au8name[] = { 'N', 'O', 'E' };
		V_PRINT(" p: set UART parity (%c)%c" LB,
					au8name[FL_IS_MODIFIED_u8(parity) ? FL_UNSAVE_u8(parity) : FL_MASTER_u8(parity)],
					FL_IS_MODIFIED_u8(parity) ? '*' : ' ');
	}

	{
		V_PRINT(" C: set crypt mode (%d)%c" LB,
					FL_IS_MODIFIED_u8(Crypt) ? FL_UNSAVE_u8(Crypt) : FL_MASTER_u8(Crypt),
					FL_IS_MODIFIED_u8(Crypt) ? '*' : ' ');
	}

	{
		V_PRINT(" K: set crypt key [%s]%c" LB,
			sAppData.sConfig_UnSaved.au8AesKey[0] == 0xFF ?
				sAppData.sFlash.sData.au8AesKey : sAppData.sConfig_UnSaved.au8AesKey,
			sAppData.sConfig_UnSaved.au8AesKey[0] == 0xFF ? ' ' : '*');
	}

	V_PRINT("---"LB);

	V_PRINT(" S: save Configuration" LB " R: reset to Defaults" LB LB);
	//       0123456789+123456789+123456789+1234567894123456789+123456789+123456789+123456789
}

/** @ingroup MASTER
 * シリアルポートからの入力を処理します。
 * - シリアルポートからの入力は uart.c/serial.c により管理される FIFO キューに値が格納されます。
 *   このキューから１バイト値を得るのが SERIAL_i16RxChar() です。
 * - 本関数では、入力したバイトに対し、アプリケーションのモードに依存した処理を行います。
 *   - 文字列入力モード時(INPSTR_ API 群、インタラクティブモードの設定値入力中)は、INPSTR_u8InputByte()
 *     API に渡す。文字列が完了したときは vProcessInputString() を呼び出し、設定値の入力処理を
 *     行います。
 *   - 上記文字列入力ではない場合は、ModBusAscii_u8Parse() を呼び出します。この関数は + + + の
 *     入力判定および : で始まる書式を認識します。
 *   - 上記書式解釈中でない場合は、vProcessInputByte() を呼び出します。この関数はインタラクティブ
 *     モードにおける１文字入力コマンドを処理します。
 *
 */
void vHandleSerialInput() {
	// handle UART command
	while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
		int16 i16Char;

		i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

		// process
		if (i16Char >=0 && i16Char <= 0xFF) {
			//DBGOUT(0, "[%02x]", i16Char);
			if (INPSTR_bActive(&sSerInpStr)) {
				// 文字列入力モード
				uint8 u8res = INPSTR_u8InputByte(&sSerInpStr, (uint8)i16Char);

				if (u8res == E_INPUTSTRING_STATE_COMPLETE) {
					vProcessInputString(&sSerInpStr);
				} else if (u8res == E_INPUTSTRING_STATE_CANCELED) {
					V_PRINT("(canceled)");
					u16HoldUpdateScreen = 64;
				}
				continue;
			}

			{
				// コマンド書式の系列解釈、および verbose モードの判定
				uint8 u8res = ModBusAscii_u8Parse(&sSerCmdIn, (uint8)i16Char);

				if (u8res != E_MODBUS_CMD_EMPTY) {
					V_PUTCHAR(i16Char);
				}

				if (u8res == E_MODBUS_CMD_COMPLETE || u8res == E_MODBUS_CMD_LRCERROR) {
					// 解釈完了

					if (u8res == E_MODBUS_CMD_LRCERROR) {
						// command complete, but CRC error
						V_PRINT(LB "!INF LRC_ERR? (might be %02X)" LB, sSerCmdIn.u8lrc);
					}

					if (u8res == E_MODBUS_CMD_COMPLETE) {
						// process command
						vProcessSerialCmd(&sSerCmdIn);
					}

					continue;
				} else
				if (u8res != E_MODBUS_CMD_EMPTY) {
					if (u8res == E_MODBUS_CMD_VERBOSE_ON) {
						// verbose モードの判定があった
						vSerUpdateScreen();
					}

					if (u8res == E_MODBUS_CMD_VERBOSE_OFF) {
						vfPrintf(&sSerStream, "!INF EXIT INTERACTIVE MODE."LB);
					}

					// still waiting for bytes.
					continue;
				} else {
					; // コマンド解釈モードではない
				}
			}

			// Verbose モードのときは、シングルコマンドを取り扱う
			if (sSerCmdIn.bverbose) {
				vProcessInputByte(i16Char);
			}
		}
	}
}

/** @ingroup MASTER
 * １バイト入力コマンドの処理\n
 * - 設定値の入力が必要な項目の場合、INPSTR_vStart() を呼び出して文字列入力モードに遷移します。
 * - フラッシュへのセーブ時の手続きでは、sAppData.sConfig_UnSaved 構造体で入力が有ったものを
 *   sFlash.sData 構造体に格納しています。
 * - デバッグ用の確認コマンドも存在します。
 *
 * @param u8Byte 入力バイト
 */
static void vProcessInputByte(uint8 u8Byte) {
	static uint8 u8lastbyte;

	switch (u8Byte) {
	case 0x0d: case 'h': case 'H':
		// 画面の書き換え
		u16HoldUpdateScreen = 1;
		break;

	case 'a': // set application ID
		V_PRINT("Input Application ID (HEX:32bit): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 8, E_APPCONF_APPID);
		break;

	case 'c': // チャネルの設定
		V_PRINT("Input Channel(s) (e.g. 11,16,21): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 8, E_APPCONF_CHMASK);
		break;

	case 'i': // set application role
		V_PRINT("Input Device ID (DEC:1-100): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 3, E_APPCONF_ID);
		break;

	case 'x': // 出力の変更
		V_PRINT("Tx Power (0[min]-3[max]): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 1, E_APPCONF_TX_POWER);
		break;

	case 't': // set application role
		V_PRINT("Input mode4 sleep dur[ms] (DEC:100-10000): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 5, E_APPCONF_SLEEP4);
		break;

	case 'y': // set application role
		V_PRINT("Input mode7 sleep dur[ms] (DEC:0-10000): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 5, E_APPCONF_SLEEP7);
		break;

	case 'f': // set application role
		V_PRINT("Input mode3 fps (DEC:4,8,16,32): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 2, E_APPCONF_FPS);
		break;

	case 'b': // ボーレートの変更
		V_PRINT("Input baud rate (DEC:9600-230400): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 10, E_APPCONF_BAUD_SAFE);
		break;

	case 'p': // パリティの変更
		V_PRINT("Input parity (N, E, O): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 1, E_APPCONF_BAUD_PARITY);
		break;

	case 'o': // オプションビットの設定
		V_PRINT("Input option bits (HEX): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_HEX, 8, E_APPCONF_OPT);
		break;

	case 'C':
		V_PRINT("Input crypt mode (0,1): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 2, E_APPCONF_CRYPT_MODE);
		break;

	case 'K':
		V_PRINT("Input crypt key: ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, 32, E_APPCONF_CRYPT_KEY);
		break;

	case 'd': // ホールドモード(マスク)
		V_PRINT("Input Hold mask (e.g. DI2,4 -> 000000001010): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_STRING, MAX_IO, E_APPCONF_HOLD_MASK);
		break;

	case 'D': // ホールド時間
		V_PRINT("Input Hold duration[ms] (DEC:20-64000): ");
		INPSTR_vStart(&sSerInpStr, E_INPUTSTRING_DATATYPE_DEC, 6, E_APPCONF_HOLD_DUR);
		break;

	case 'S':
		// フラッシュへのデータ保存
		if (u8lastbyte == 'R') {
			// R S と連続入力された場合は、フラッシュエリアを消去する
			V_PRINT("!INF CLEAR SAVE AREA.");
			bFlash_Erase(FLASH_SECTOR_NUMBER - 1); // SECTOR ERASE

			vWait(1000000);
			vAHI_SwReset();
		} else {
			vConfig_SaveAndReset();
		}
		break;

	case 'R':
		vConfig_SetDefaults(&sAppData.sConfig_UnSaved);
		u16HoldUpdateScreen = 1;
		break;

	case '$':
		sAppData.u8DebugLevel++;
		if(sAppData.u8DebugLevel > 5) sAppData.u8DebugLevel = 0;

		V_PRINT("* set App debug level to %d." LB, sAppData.u8DebugLevel);
		break;

	case '@':
		_C {
			static uint8 u8DgbLvl;

			u8DgbLvl++;
			if(u8DgbLvl > 5) u8DgbLvl = 0;
			ToCoNet_vDebugLevel(u8DgbLvl);

			V_PRINT("* set NwkCode debug level to %d." LB, u8DgbLvl);
		}
		break;

	case '!':
		// リセット
		V_PRINT("!INF RESET SYSTEM.");
		vWait(1000000);
		vAHI_SwReset();
		break;

	case '#': // info
		_C {
			V_PRINT("*** ToCoNet(ver%08X) ***" LB, ToCoNet_u32GetVersion());
			V_PRINT("* AppID %08x, LongAddr, %08x, ShortAddr %04x, Tk: %d" LB,
					sToCoNet_AppContext.u32AppId, ToCoNet_u32GetSerial(), sToCoNet_AppContext.u16ShortAddress, u32TickCount_ms);
			if (sAppData.bFlashLoaded) {
				V_PRINT("** Conf "LB);
				V_PRINT("* AppId = %08x"LB, sAppData.sFlash.sData.u32appid);
				V_PRINT("* ChMsk = %08x"LB, sAppData.sFlash.sData.u32chmask);
				V_PRINT("* Ch=%d, Role=%d, Layer=%d"LB,
						sToCoNet_AppContext.u8Channel,
						sAppData.sFlash.sData.u8role,
						sAppData.sFlash.sData.u8layer);
			} else {
				V_PRINT("** Conf: none"LB);
			}
		}
		break;

	case 'V':
		vSerInitMessage();
		V_PRINT("---"LB);
		V_PRINT("ToCoNet lib version Core: %08x, Ext: %08x, Utils: %08x"LB,
				ToCoNet_u32GetVersion(),
				ToCoNet_u32GetVersion_LibEx(),
				ToCoNet_u32GetVersion_LibUtils());
		V_PRINT("ToCoNet Tick Counter: %d"LB, u32TickCount_ms);
		V_PRINT("Run Time: %d+%02d/64 sec"LB, sAppData.u32CtTimer0 >> 6, sAppData.u32CtTimer0 & 0x3F);
		V_PRINT(""LB);
		break;

#ifdef USE_I2C_LCD_TEST_CODE
	case '1':
	case '2':
	case '3':
	case '4':
		_C {
			bool_t bRes;
#if defined(USE_I2C_AQM1602)
			bRes = bDraw2LinesLcd_ACM1602(astrLcdMsgs[u8Byte-'1'][0], astrLcdMsgs[u8Byte-'1'][1]);
#elif defined(USE_I2C_AQM0802A)
			bRes = bDraw2LinesLcd_AQM0802A(astrLcdMsgs[u8Byte-'1'][0], astrLcdMsgs[u8Byte-'1'][1]);
#endif
			V_PRINT("I2C LCD = %d: %s,%s"LB, bRes, astrLcdMsgs[u8Byte-'1'][0], astrLcdMsgs[u8Byte-'1'][1]);
		}

		break;
#endif

	default:
		u8lastbyte = 0xFF;
		break;
	}

	// 一つ前の入力
	if (u8lastbyte != 0xFF) {
		u8lastbyte = u8Byte;
	}

}

/** @ingroup MASTER
 * 文字列入力モードの処理を行います。
 *
 */
static void vProcessInputString(tsInpStr_Context *pContext) {
	uint8 *pu8str = pContext->au8Data;
	uint8 u8idx = pContext->u8Idx;

	switch(pContext->u32Opt) {
	case E_APPCONF_APPID:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);

			uint16 u16h, u16l;
			u16h = u32val >> 16;
			u16l = u32val & 0xFFFF;

			if (u16h == 0x0000 || u16h == 0xFFFF || u16l == 0x0000 || u16l == 0xFFFF) {
				V_PRINT("(ignored: 0x0000????,0xFFFF????,0x????0000,0x????FFFF can't be set.)");
			} else {
				sAppData.sConfig_UnSaved.u32appid = u32val;
			}

			V_PRINT(LB"-> %08X"LB, u32val);
		}
		break;

	case E_APPCONF_CHMASK:
		_C {
			// チャネルマスク（リスト）を入力解釈する。
			//  11,15,19 のように１０進数を区切って入力する。
			//  以下では区切り文字は任意で MAX_CHANNELS 分処理したら終了する。

			uint8 b = 0, e = 0, i = 0, n_ch = 0;
			uint32 u32chmask = 0; // 新しいチャネルマスク

			V_PRINT(LB"-> ");

			for (i = 0; i <= pContext->u8Idx; i++) {
				if (pu8str[i] < '0' || pu8str[i] > '9') {
					e = i;
					uint8 u8ch = 0;

					// 最低２文字あるときに処理
					if (e - b > 0) {
						u8ch = u32string2dec(&pu8str[b], e - b);
						if (u8ch >= 11 && u8ch <= 26) {
							if (n_ch) {
								V_PUTCHAR(',');
							}
							V_PRINT("%d", u8ch);
							u32chmask |= (1UL << u8ch);

							n_ch++;
							if (n_ch >= MAX_CHANNELS) {
								break;
							}
						}
					}
					b = i + 1;
				}

				if (pu8str[i] == 0x0) {
					break;
				}
			}

			if (u32chmask == 0x0) {
				V_PRINT("(ignored)");
			} else {
				sAppData.sConfig_UnSaved.u32chmask = u32chmask;
			}

			V_PRINT(LB);
		}
		break;

	case E_APPCONF_ID:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val <= 0x7F) {
				sAppData.sConfig_UnSaved.u8id = u32val;
				V_PRINT("%d(0x%02x)"LB, u32val, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_TX_POWER:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val <= 3) {
				sAppData.sConfig_UnSaved.u8pow = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_SLEEP4:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);
			V_PRINT(LB"-> ");
			if (u32val >= 100 && u32val <= 65534) {
				sAppData.sConfig_UnSaved.u16SleepDur_ms = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_SLEEP7:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);

			V_PRINT(LB"-> ");
			if (u32val <= 65534) { // 0 はタイマーを使用しない (
				sAppData.sConfig_UnSaved.u16SleepDur_s = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_FPS:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);

			V_PRINT(LB"-> ");
			if (u32val == 4 || u32val == 8 || u32val == 16 || u32val == 32) {
				sAppData.sConfig_UnSaved.u8Fps = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_OPT:
		_C {
			uint32 u32val = u32string2hex(pu8str, u8idx);

			V_PRINT(LB"-> ");

			sAppData.sConfig_UnSaved.u32Opt = u32val;
			V_PRINT("0x%08X"LB, u32val);
		}
		break;

	case E_APPCONF_BAUD_SAFE:
		_C {
			uint32 u32val = 0;

			if (pu8str[0] == '0' && pu8str[1] == 'x') {
				u32val = u32string2hex(pu8str + 2, u8idx - 2);
			} if (u8idx <= 6) {
				u32val = u32string2dec(pu8str, u8idx);
			}

			V_PRINT(LB"-> ");

			if (u32val) {
				sAppData.sConfig_UnSaved.u32baud_safe = u32val;
				if (u32val & 0x80000000) {
					V_PRINT("%x"LB, u32val);
				} else {
					V_PRINT("%d"LB, u32val);
				}
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_BAUD_PARITY:
		_C {
			V_PRINT(LB"-> ");

			if (pu8str[0] == 'N' || pu8str[0] == 'n') {
				sAppData.sConfig_UnSaved.u8parity = 0;
				V_PRINT("None"LB);
			} else if (pu8str[0] == 'O' || pu8str[0] == 'o') {
				sAppData.sConfig_UnSaved.u8parity = 1;
				V_PRINT("Odd"LB);
			} else if (pu8str[0] == 'E' || pu8str[0] == 'e') {
				sAppData.sConfig_UnSaved.u8parity = 2;
				V_PRINT("Even"LB);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_CRYPT_MODE:
		_C {
			if (pu8str[0] == '0') {
				sAppData.sConfig_UnSaved.u8Crypt = 0;
				V_PRINT(LB"--> Plain");
			} else if (pu8str[0] == '1') {
				sAppData.sConfig_UnSaved.u8Crypt = 1;
				V_PRINT(LB"--> AES128");
			} else {
				V_PRINT(LB"(ignored)");
			}
		}
		break;

	case E_APPCONF_CRYPT_KEY:
		_C {
			uint8 u8len = strlen((void*)pu8str);

			if (u8len == 0) {
				memset(sAppData.sConfig_UnSaved.au8AesKey, 0, sizeof(sAppData.sConfig_UnSaved.au8AesKey));
				V_PRINT(LB"(cleared)");
			} else
			if (u8len && u8len <= 32) {
				memset(sAppData.sConfig_UnSaved.au8AesKey, 0, sizeof(sAppData.sConfig_UnSaved.au8AesKey));
				memcpy(sAppData.sConfig_UnSaved.au8AesKey, pu8str, u8len);
				V_PRINT(LB);
			} else {
				V_PRINT(LB"(ignored)"LB);
			}
		}
		break;

	case E_APPCONF_HOLD_MASK:
		_C {
			uint8 u8len = strlen((void*)pu8str);
			uint32 u32mask = 0;
			int i = 0;

			for (i = 0; i < u8len && i < sAppData.u8MaxIoCount; i++) {
				if (pu8str[u8len - i - 1] == '1') {
					u32mask |= (1UL << i);
				}
			}

			sAppData.sConfig_UnSaved.u32HoldMask = u32mask;
			V_PRINT(LB "-->%012b", u32mask);
		}
		break;

	case E_APPCONF_HOLD_DUR:
		_C {
			uint32 u32val = u32string2dec(pu8str, u8idx);

			V_PRINT(LB"-> ");
			if (u32val >= 20 && u32val <= 65534) { // 0 はタイマーを使用しない (
				sAppData.sConfig_UnSaved.u16HoldDur_ms = u32val;
				V_PRINT("%d"LB, u32val);
			} else {
				V_PRINT("(ignored)"LB);
			}
		}
		break;

	default:
		break;
	}

	// 一定時間待って画面を再描画
	u16HoldUpdateScreen = 96; // 1.5sec
}

/** @ingroup MASTER
 * シリアルから入力されたコマンド形式の電文を処理します。
 *
 * - 先頭バイトがアドレス指定で、0xDB 指定の場合、自モジュールに対しての指令となります。
 * - ２バイト目がコマンドで、0x80 以降を指定します。0x7F 以下は特別な処理は割り当てられていません。
 * - コマンド(0xDB向け)
 *   - SERCMD_ID_GET_MODULE_ADDRESS\n
 *     モジュールのシリアル番号を取得する
 * - コマンド(外部アドレス向け)
 *   - SERCMD_ID_REQUEST_IO_DATA\n
 *     IO状態の設定
 *   - それ以外のコマンドID\n
 *     通常送信 (ただし 0x80 以降は今後の機能追加で意味が変わる可能性あり)
 *
 * @param pSer シリアルコマンド入力の管理構造体
 */
static void vProcessSerialCmd(tsModbusCmd *pSer) {
	uint8 *p = pSer->au8data;

	uint8 u8addr; // 送信先論理アドレス
	uint8 u8cmd; // コマンド

	uint8 *p_end;
	p_end = &(pSer->au8data[pSer->u16len]); // the end points 1 byte after the data end.

	// COMMON FORMAT
	OCTET(u8addr); // [1] OCTET : 論理ID
	OCTET(u8cmd); // [1] OCTET : 要求番号

	DBGOUT(3, "* UARTCMD ln=%d cmd=%02x req=%02x %02x%0x2%02x%02x..." LB,
			pSer->u16len,
			u8addr,
			u8cmd,
			*p,
			*(p+1),
			*(p+2),
			*(p+3)
			);

	if (u8addr == SERCMD_ADDR_TO_MODULE) {
		/*
		 * モジュール自身へのコマンド (0xDB)
		 */
		switch(u8cmd) {
		case SERCMD_ID_GET_MODULE_ADDRESS:
			vModbOut_MySerial(&sSerStream);
			break;

		default:
			break;
		}
	} else {
		/*
		 * 外部アドレスへの送信(IO情報の設定要求)
		 */
		if (u8cmd == SERCMD_ID_REQUEST_IO_DATA) {
			/*
			 * OCTET: 書式 (0x01)
			 * OCTET: 出力状態
			 * OCTET: 出力状態マスク
			 */
			uint8 u8format = G_OCTET();

			if (u8format == 0x01) {
				tsIOSetReq sIOreq;
				memset(&sIOreq, 0, sizeof(tsIOSetReq));

				sIOreq.u16IOports = G_BE_WORD();
				sIOreq.u16IOports_use_mask = G_BE_WORD();

				int i;

				for (i = 0; i < 4; i++) {
					(void)G_BE_WORD(); // TODO
				}

				if (p > p_end) return; // v1.1.3 (終端チェック)

				DBGOUT(1, "SERCMD:IO REQ: %02x %02x"LB,
						sIOreq.u16IOports,
						sIOreq.u16IOports_use_mask
						);

				i16TransmitIoSettingRequest(u8addr, &sIOreq);
			}

			return;
		} else {
			int ilen = p_end - p;

			if (ilen > 0 && ilen <= 80) {
				i16TransmitSerMsg(u8addr, u8cmd, p, p_end - p);
			} else {
				DBGOUT(3, "* SERMSG: invalid length (%d)", ilen);
			}
		}
	}
}

/** @ingroup MASTER
 * 重複パケットの判定。タイムスタンプの比較で、ごく最近であって旧いパケットを除外する。
 *
 * - 注意点
 *   - DUPCHK_vInin() の初期化を行うこと
 *   - DUPCHK_bFIND(0,NULL) を一定周期で呼び出すこと
 *
 * @param pc 管理構造体
 * @param u32Addr
 * @param u16TimeStamp
 * @return TRUE:重複している FALSE:新しいパケットが来た
 */
static bool_t bCheckDupPacket(tsDupChk_Context *pc, uint32 u32Addr, uint16 u16TimeStamp) {
	uint32 u32Key;
	if (DUPCHK_bFind(pc, u32Addr, &u32Key)) {
		// 最後に受けたカウンタの値が得られるので、これより新しい
		uint16 u16Delta = ((uint16)u32Key - u16TimeStamp) & 0x7FFF; // 最上位ビットは設定されない
		if (u16Delta < 32) { // 32count=500ms, 500ms の遅延は想定外だが。
			// すでに処理したパケット
			return TRUE;
		}
	}

	// 新しいパケットである（時間情報を格納する）
	DUPCHK_vAdd(pc, u32Addr, u16TimeStamp);
	return FALSE;
}

/** @ingroup MASTER
 * IO 情報を送信します。
 *
 * - IO状態の変化、および１秒置きの定期送時に呼び出されます。
 *
 * - Packet 構造
 *   - OCTET: 識別ヘッダ(APP ID より生成)
 *   - OCTET: プロトコルバージョン(バージョン間干渉しないための識別子)
 *   - OCTET: 送信元論理ID
 *   - BE_DWORD: 送信元のシリアル番号
 *   - OCTET: 宛先論理ID
 *   - BE_WORD: 送信タイムスタンプ (64fps カウンタの値の下１６ビット, 約1000秒で１周する)
 *   - OCTET: 中継フラグ(中継したパケットは１、最初の送信なら０を格納）
 *   - BE_WORD: ポート状態 (LSB から順に I1, I2, ... I12, 1=Lo)
 *   - BE_WORD: 設定ポート (LSB から順に I1, I2, ... I12, 1=変化) 本値が１のポートを設定対象とする
 *
 * @param bQuick TRUEなら遅延無しで送信しようと試みる
 * @returns -1:ERROR, 0..255 CBID
 */
static int16 i16TransmitIoData(bool_t bQuick) {
	if(IS_APPCONF_ROLE_SILENT_MODE()) return -1;

	int16 i16Ret = -1;
	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));

	uint8 *q = sTx.auData;

	//int i;

	// ペイロードを構成
	S_OCTET(sAppData.u8AppIdentifier);
	S_OCTET(APP_PROTOCOL_VERSION);
	S_OCTET(sAppData.u8AppLogicalId); // アプリケーション論理アドレス
	S_BE_DWORD(ToCoNet_u32GetSerial());  // シリアル番号
	S_OCTET(IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId) ? LOGICAL_ID_CHILDREN : LOGICAL_ID_PARENT); // 宛先
	S_BE_WORD((sAppData.u32CtTimer0 & 0x7FFF) + (bQuick ? 0x8000 : 0)); // タイムスタンプ
		// bQuick 転送する場合は MSB をセットし、優先パケットである処理を行う
	S_OCTET(0); // 中継フラグ

	// IOの状態
	{
		int i;
		uint16 u16bm = 0;

		for (i = 0; i < sAppData.u8MaxIoCount; i++) {
			uint8 u8ct = sAppData.sIOData_now.au8Input[i] >> 4; // カウンタ値の取り出し

			if (u8ct >= LOW_LATENCY_DELAYED_TRANSMIT_COUNTER - 3) {
				// カウンタ値が残っている間は１とする(チャタリングが起きているかもしれないので、押したと判断する）
				u16bm |= (1 << i);
			} else {
				// カウンタ値残存期間が過ぎたら BTM による値を採用する
				u16bm |= (sAppData.sIOData_now.au8Input[i] & 1) ? (1 << i) : 0;
			}
		}

		S_BE_WORD(u16bm);

	// IOの使用フラグ (一度でも設定したポートは対象とする)
		uint16 u16bm_used = 0;
		for (i = 0; i < sAppData.u8MaxIoCount; i++) {
			if (sAppData.sIOData_now.u32BtmUsed & (1UL << au8PortTbl_DIn[i])) {
				u16bm_used |= (1UL << i);
			}
		}

		u16bm_used |= u16bm; // IO状態で押下げにあるものは無条件で使用
				// （スリープ直後に強制的に値を入れて送信したい時に割り込み？で値がセットされてしまう）

		S_BE_WORD(u16bm_used);
	}

	// 送信情報の構築
	sTx.u8Len = q - sTx.auData; // パケット長
	sTx.u8Cmd = TOCONET_PACKET_CMD_APP_USER_IO_DATA; // パケット種別

	if (IS_APPCONF_OPT_ACK_MODE() && IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
		// Ackモード時
		sTx.u32DstAddr  = SERCMD_ADDR_CONV_TO_SHORT_ADDR(LOGICAL_ID_PARENT); // 親機宛に送信(Ack付き)
		sTx.u8Retry     = 1; // アプリ再送は１回のみ
		sTx.bAckReq = TRUE;
	} else {
		sTx.u32DstAddr  = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
		sTx.u8Retry     = 0x82; // 1回再送
		sTx.bAckReq = FALSE;
	}

	// フレームカウントとコールバック識別子の指定
	sAppData.u16TxFrame++;
	sTx.u8Seq = (sAppData.u16TxFrame & 0xFF);
	sTx.u8CbId = sTx.u8Seq;

	{
		/* MAC モードでは細かい指定が可能 */
		sTx.bSecurePacket = IS_CRYPT_MODE() ? TRUE : FALSE;
		sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
		sTx.u16RetryDur = bQuick ? 0 : 4; // 再送間隔
		sTx.u16DelayMax = bQuick ? 0 : 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)

		// 送信API
		if (ToCoNet_bMacTxReq(&sTx)) {
			i16Ret = sTx.u8CbId;
			sAppData.sIOData_now.u32TxLastTick = u32TickCount_ms;
		}
	}

	return i16Ret;
}

/** @ingroup MASTER
 * IOデータを中継送信します。
 *
 * - パケット中の中継フラグのビットは、呼び出し前に設定されています。
 * - 衝突を抑制出来る程度の送信遅延、再送間隔を設定しています。
 *
 * @param pRx 受信したときのデータ
 * @return -1:Error, 0..255:CbId
 */
int16 i16TransmitRepeat(tsRxDataApp *pRx) {
	if(IS_APPCONF_ROLE_SILENT_MODE()) return -1;

	int16 i16Ret = -1;

	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));

	// Payload
	memcpy(sTx.auData, pRx->auData, pRx->u8Len);
	sTx.u8Len = pRx->u8Len;

	// コマンド設定
	sTx.u8Cmd = pRx->u8Cmd; // パケット種別

	// 送信する
	sTx.u32DstAddr  = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
	sTx.u8Retry     = 0x82; // ２回再送

	// フレームカウントとコールバック識別子の指定
	sAppData.u16TxFrame++;
	sTx.u8Seq = (sAppData.u16TxFrame & 0xFF);
	sTx.u8CbId = sTx.u8Seq;

	// 中継時の送信パラメータ
	sTx.bAckReq = FALSE;
	sTx.bSecurePacket = IS_CRYPT_MODE() ? TRUE : FALSE;
	sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
	sTx.u16RetryDur = 8; // 再送間隔

	sTx.u16DelayMin = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)
	sTx.u16DelayMax = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)

	// 送信API
	if (ToCoNet_bMacTxReq(&sTx)) {
		i16Ret = sTx.u8CbId;
	}

	return i16Ret;
}


/** @ingroup MASTER
 * IO(DO/PWM)を設定する要求コマンドパケットを送信します。
 *
 * - Packet 構造
 *   - OCTET: 識別ヘッダ(APP ID より生成)
 *   - OCTET: プロトコルバージョン(バージョン間干渉しないための識別子)
 *   - OCTET: 送信元論理ID
 *   - BE_DWORD: 送信元のシリアル番号
 *   - OCTET: 宛先論理ID
 *   - BE_WORD: 送信タイムスタンプ (64fps カウンタの値の下１６ビット, 約1000秒で１周する)
 *   - OCTET: 中継フラグ
 *   - OCTET: 形式 (1固定)
 *   - OCTET: ボタン (LSB から順に SW1 ... SW4, 1=Lo)
 *   - OCTET: ボタン使用フラグ (LSB から順に SW1 ... SW4, 1=このポートを設定する)
 *
 * @param u8DstAddr 送信先
 * @param pReq 設定データ
 * @return -1:Error, 0..255:CbId
 */
int16 i16TransmitIoSettingRequest(uint8 u8DstAddr, tsIOSetReq *pReq) {
	if(IS_APPCONF_ROLE_SILENT_MODE()) return -1;

	int16 i16Ret;

	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));

	uint8 *q = sTx.auData;

	// ペイロードの構築
	S_OCTET(sAppData.u8AppIdentifier);
	S_OCTET(APP_PROTOCOL_VERSION);
	S_OCTET(sAppData.u8AppLogicalId); // アプリケーション論理アドレス
	S_BE_DWORD(ToCoNet_u32GetSerial());  // シリアル番号
	S_OCTET(u8DstAddr); // 宛先
	S_BE_WORD(sAppData.u32CtTimer0 & 0xFFFF); // タイムスタンプ
	S_OCTET(0); // 中継フラグ

	S_OCTET(1); // パケット形式

	// DIO の設定
	S_BE_WORD(pReq->u16IOports);
	S_BE_WORD(pReq->u16IOports_use_mask);

	// 各種送信情報
	sTx.u8Len = q - sTx.auData; // パケット長
	sTx.u8Cmd = TOCONET_PACKET_CMD_APP_USER_IO_DATA_EXT; // パケット種別

	// 送信する
	sTx.u32DstAddr  = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
	sTx.u8Retry     = 0x82; // 1回再送

	{
		/* 送信設定 */
		sTx.bAckReq = FALSE;
		sTx.bSecurePacket = IS_CRYPT_MODE() ? TRUE : FALSE;
		sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress;
		sTx.u16RetryDur = 4; // 再送間隔
		sTx.u16DelayMax = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)

		// 送信API
		if (ToCoNet_bMacTxReq(&sTx)) {
			i16Ret = sTx.u8CbId;
		}
	}

	return i16Ret;
}

/** @ingroup MASTER
 * シリアルコマンドを送信します
 *
 * - Packet 構造
 *   - OCTET: 識別ヘッダ(APP ID より生成)
 *   - OCTET: プロトコルバージョン(バージョン間干渉しないための識別子)
 *   - OCTET: 送信元論理ID
 *   - BE_DWORD: 送信元のシリアル番号
 *   - OCTET: 宛先論理ID
 *   - BE_WORD: 送信タイムスタンプ (64fps カウンタの値の下１６ビット, 約1000秒で１周する)
 *   - OCTET: 中継フラグ
 *   - OCTET: コマンド名
 *   - OCTET[]: データ
 *
 * @param u8DstAddr
 * @param pDat 送信データ
 * @param u8len 送信データ長
 * @return -1:Error, 0..255:CbId
 */
static int16 i16TransmitSerMsg(uint8 u8DstAddr, uint8 u8Cmd, uint8 *pDat, uint8 u8len) {
	if(IS_APPCONF_ROLE_SILENT_MODE()) return -1;
	if (u8len > 80) { return -1; }

	int16 i16Ret;

	tsTxDataApp sTx;
	memset(&sTx, 0, sizeof(sTx));

	uint8 *q = sTx.auData;

	// ペイロードの構築
	S_OCTET(sAppData.u8AppIdentifier);
	S_OCTET(APP_PROTOCOL_VERSION);
	S_OCTET(sAppData.u8AppLogicalId); // アプリケーション論理アドレス
	S_BE_DWORD(ToCoNet_u32GetSerial());  // シリアル番号
	S_OCTET(u8DstAddr); // 宛先
	S_BE_WORD(sAppData.u32CtTimer0 & 0xFFFF); // タイムスタンプ
	S_OCTET(0); // 中継フラグ

	S_OCTET(u8Cmd);
	memcpy(q, pDat, u8len); q += u8len; // データ中身

	// 各種送信情報
	sTx.u8Len = q - sTx.auData; // パケット長
	sTx.u8Cmd = TOCONET_PACKET_CMD_APP_USER_SERIAL_MSG; // パケット種別

	// 送信する
	sTx.u32DstAddr  = TOCONET_MAC_ADDR_BROADCAST; // ブロードキャスト
	sTx.u8Retry     = 0x82; // 1回再送

	{
		/* 送信設定 */
		sTx.bAckReq = FALSE;
		sTx.bSecurePacket = IS_CRYPT_MODE() ? TRUE : FALSE;
		sTx.u32SrcAddr = sToCoNet_AppContext.u16ShortAddress; // ペイロードにアドレスが含まれるのでショートアドレス指定
		sTx.u16RetryDur = 4; // 再送間隔
		sTx.u16DelayMax = 16; // 衝突を抑制するため送信タイミングにブレを作る(最大16ms)

		// 送信API
		if (ToCoNet_bMacTxReq(&sTx)) {
			i16Ret = sTx.u8CbId;
		}
	}

	return i16Ret;
}

/** @ingroup MASTER
 * IO状態パケットの受信処理を行います。
 *
 * - 受信したデータに格納されるIO設定要求に従いIO値(DO/PWM)を設定します。
 * - 受信したデータを UART に出力します。
 * - 中継機の場合、中継パケットを送信します。
 * - 低遅延で送信されてきたパケットの TimeStamp の MSB には１が設定される。
 *   このパケットは、タイムスタンプによる重複除去アルゴリズムとは独立して
 *   処理される。
 * - IOの設定は本受信関数でのみ行われる。
 * - UART 出力書式
 *   OCTET: 送信元ID
 *   OCTET: 0x81
 *   OCTET: ペイロードの長さ(このバイトも含む)
 *   OCTET: 0x01
 *   OCTET: LQI値
 *   BE_DWORD: 送信元アドレス
 *   OCTET: 送信先アドレス
 *   BW_WORD: タイムスタンプ (MSB 0x8000 が設定されているときはクイック送信)
 *   OCTET: 中継フラグ (中継されていないパケットなら 0)
 *   BE_WORD: ボタン状態 (1=Lo, 0=Hi, LSB から順に DI1...)
 *            ボタンの有効マスクが 1 のビットに限り意味のある情報となる。
 *   BE_WORD: ボタンの有効マスク (1=有効 0=無効, LSBから順に DI1...)
 *
 * @param pRx 受信データ
 */
static void vReceiveIoData(tsRxDataApp *pRx) {
	int i, j;
	uint8 *p = pRx->auData;

	uint8 u8AppIdentifier = G_OCTET();
	if (u8AppIdentifier != sAppData.u8AppIdentifier) return;

	uint8 u8PtclVersion = G_OCTET();
	if (u8PtclVersion != APP_PROTOCOL_VERSION) return;

	uint8 u8AppLogicalId = G_OCTET();

	uint32 u32Addr = G_BE_DWORD();

	uint8 u8AppLogicalId_Dest = G_OCTET(); (void)u8AppLogicalId_Dest;

	uint16 u16TimeStamp = G_BE_WORD();

	/* 重複の確認を行う */
	bool_t bQuick = u16TimeStamp & 0x8000 ? TRUE : FALSE; // 優先パケット（全部処理する）
	u16TimeStamp &= 0x7FFF;
	if (bQuick == FALSE && bCheckDupPacket(&sDupChk_IoData, u32Addr, u16TimeStamp)) {
		return;
	}
	static uint32 u32LastQuick;
	if (bQuick) {
		if ((u32TickCount_ms - u32LastQuick) < 20) {
			// Quickパケットを受けて一定期間未満のパケットは無視する
			return;
		} else {
			u32LastQuick = u32TickCount_ms; // タイムスタンプを巻き戻す
		}
	}

	// 中継フラグ
	uint8 u8TxFlag = G_OCTET();
	// 中継の判定 (このフラグが１なら中継済みのパケット）
	if (u8TxFlag == 0 && sAppData.u8Mode == E_IO_MODE_ROUTER) {

		// リピータの場合はここで中継の判定
		*(p-1) = 1; // 中継済みフラグのセット

		// 中継する
		i16TransmitRepeat(pRx);
		return;
	}

	// 親機子機の判定
	bool_t bSetIo = FALSE;
	if (	(IS_LOGICAL_ID_PARENT(u8AppLogicalId) && IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId))
		||	(IS_LOGICAL_ID_CHILD(u8AppLogicalId) && IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) ) {
		bSetIo = TRUE; // 親機⇒子機、または子機⇒親機への伝送
	}

	/* BUTTON */
	uint16 u16ButtonState = G_BE_WORD();
	uint16 u16ButtonChanged = G_BE_WORD();

	if (bSetIo) {
		// ポートの値を設定する（変更フラグのあるものだけ）
		for (i = 0, j = 1; i < sAppData.u8MaxIoCount; i++, j <<= 1) {
			if (u16ButtonChanged & j) {
				if (   ((j & sAppData.sFlash.sData.u32HoldMask) && !IS_APPCONF_OPT_ON_PRESS_TRANSMIT()) // 通常モード時は、対象ポートをホールドする
					|| (!(j & sAppData.sFlash.sData.u32HoldMask) && IS_APPCONF_OPT_ON_PRESS_TRANSMIT()) // OnPressTransmit モードでは、対象ポート以外はホールドする
				) {
					// hold モードの処理
					if (u16ButtonState & j) {
						// Hi>Loへの遷移のみ対象とする
						MICRO_INT_STORAGE;
						MICRO_INT_ENABLE_ONLY(0); // 割り込み禁止
						sAppData.au16HoldBtn[i] = IS_APPCONF_OPT_ON_PRESS_TRANSMIT()
								? 50 : sAppData.sFlash.sData.u16HoldDur_ms;
								// ホールド時間を設定(割り込みハンドラでHiの制御を実施)
								// ON_PRESS_TRANSMIT 時で、対象ポートでない場合は 50ms 固定とする(mode7 の親和性を優先)
						vPortSetLo(au8PortTbl_DOut[i]); // Lo に落とす
						MICRO_INT_RESTORE_STATE(); // 割り込み解除
					}
				} else {
					vPortSet_TrueAsLo(au8PortTbl_DOut[i], u16ButtonState & j);
				}
				sAppData.sIOData_now.au8Output[i] = u16ButtonState & j;
			}
		}

		// タイムスタンプの保存
		sAppData.sIOData_now.u32RxLastTick = u32TickCount_ms;
	}

	/* UART 出力 */
	if (!sSerCmdIn.bverbose && !IS_APPCONF_OPT_CHILD_RECV_NO_IO_DATA()) {
		// 以下のようにペイロードを書き換えて UART 出力
		pRx->auData[0] = pRx->u8Len; // １バイト目はバイト数
		pRx->auData[2] = pRx->u8Lqi; // ３バイト目(もともとは送信元の LogicalID) は LQI

		vSerOutput_ModbusAscii(&sSerStream, u8AppLogicalId, SERCMD_ID_INFORM_IO_DATA, pRx->auData, pRx->u8Len);
	}
}

/** @ingroup MASTER
 * IO状態の設定要求を行う UART メッセージから送信されてきたパケットの処理を行います。
 * vReceiveIoData() と大まかな処理は同じですが、PWMの設定に違いが有るので独立した
 * 関数に記述しています。
 *
 * @param pRx 受信したときのデータ
 */
static void vReceiveIoSettingRequest(tsRxDataApp *pRx) {
	int i, j;
	uint8 *p = pRx->auData;

	uint8 u8AppIdentifier = G_OCTET();
	if (u8AppIdentifier != sAppData.u8AppIdentifier) return;

	uint8 u8PtclVersion = G_OCTET();
	if (u8PtclVersion != APP_PROTOCOL_VERSION) return;

	uint8 u8AppLogicalId = G_OCTET();

	uint32 u32Addr = G_BE_DWORD();

	uint8 u8AppLogicalId_Dest = G_OCTET();

	uint16 u16TimeStamp = G_BE_WORD();

	/* 重複の確認を行う */
	if (bCheckDupPacket(&sDupChk_IoData, u32Addr, u16TimeStamp)) {
		return;
	}

	uint8 u8TxFlag = G_OCTET();

	// 中継の判定 (このフラグが１なら中継済みのパケット）
	if (u8TxFlag == 0 && sAppData.u8Mode == E_IO_MODE_ROUTER) {
		// リピータの場合はここで中継の判定
		*(p-1) = 1; // 中継済みフラグのセット

		i16TransmitRepeat(pRx);
		return;
	}

	// 親機子機の判定
	if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
		// 子機の場合は、任意の送り主から受けるが、送り先が CHILDREN(120) またはアドレスが一致している事
		if (!(u8AppLogicalId_Dest == sAppData.u8AppLogicalId || u8AppLogicalId_Dest == LOGICAL_ID_CHILDREN)) {
			return;
		}
	} else if(IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
		// 親機の場合は、子機からの送信である事
		if (!(u8AppLogicalId_Dest == LOGICAL_ID_PARENT && IS_LOGICAL_ID_CHILD(u8AppLogicalId))) {
			return;
		}
	} else {
		// それ以外は処理しない
		return;
	}

	/* 書式 */
	uint8 u8Format = G_OCTET();

	if (u8Format == 1) {
		/* BUTTON */
		uint16 u16ButtonState = G_BE_WORD();
		uint16 u16ButtonChanged = G_BE_WORD();
		// ポートの値を設定する（変更フラグのあるものだけ）
		for (i = 0, j = 1; i < sAppData.u8MaxIoCount; i++, j <<= 1) {
			if (u16ButtonChanged & j) {
				vPortSet_TrueAsLo(au8PortTbl_DOut[i], u16ButtonState & j);
				sAppData.sIOData_now.au8Output[i] = u16ButtonState & j;
			}
		}

		DBGOUT(1, "RECV:IO REQ: %02x %02x"LB,
				u16ButtonState,
				u16ButtonChanged
				);
	}
}

/** @ingroup MASTER
 * IO状態の設定要求を行う UART メッセージから送信されてきたパケットの処理を行います。
 * vReceiveIoData() と大まかな処理は同じですが、PWMの設定に違いが有るので独立した
 * 関数に記述しています。
 *
 * @param pRx 受信したときのデータ
 */
static void vReceiveSerialMsg(tsRxDataApp *pRx) {
	uint8 *p = pRx->auData;
	uint8 *p_end = pRx->auData + pRx->u8Len;

	uint8 u8AppIdentifier = G_OCTET();
	if (u8AppIdentifier != sAppData.u8AppIdentifier) return;

	uint8 u8PtclVersion = G_OCTET();
	if (u8PtclVersion != APP_PROTOCOL_VERSION) return;

	uint8 u8AppLogicalId = G_OCTET();

	uint32 u32Addr = G_BE_DWORD();

	uint8 u8AppLogicalId_Dest = G_OCTET();

	uint16 u16TimeStamp = G_BE_WORD();

	/* 重複の確認を行う */
	if (bCheckDupPacket(&sDupChk_IoData, u32Addr, u16TimeStamp)) {
		return;
	}

	uint8 u8TxFlag = G_OCTET();

	// 中継の判定 (このフラグが１なら中継済みのパケット）
	if (u8TxFlag == 0 && sAppData.u8Mode == E_IO_MODE_ROUTER) {
		// リピータの場合はここで中継の判定
		*(p-1) = 1; // 中継済みフラグのセット

		i16TransmitRepeat(pRx);
		return;
	}

	// 親機子機の判定
	if (IS_LOGICAL_ID_CHILD(sAppData.u8AppLogicalId)) {
		// 子機の場合は、任意の送り主から受けるが、送り先が CHILDREN(120) またはアドレスが一致している事
		if (!(u8AppLogicalId_Dest == sAppData.u8AppLogicalId || u8AppLogicalId_Dest == LOGICAL_ID_CHILDREN)) {
			return;
		}
	} else if(IS_LOGICAL_ID_PARENT(sAppData.u8AppLogicalId)) {
		// 親機の場合は、子機からの送信である事
		if (!(u8AppLogicalId_Dest == LOGICAL_ID_PARENT && IS_LOGICAL_ID_CHILD(u8AppLogicalId))) {
			return;
		}
	} else {
		// それ以外は処理しない
		return;
	}

	/* ペイロードの処理 */
	uint8 u8Cmd = G_OCTET();

	if (p < p_end) {
		uint8 u8paylen = p_end - p;
		if (u8Cmd == SERCMD_ID_SERCMD_EX_SIMPLE) {
			uint8 au8buff2[256], *q = au8buff2;

			S_OCTET(0x00); // 応答ID（指定なし）
			S_BE_DWORD(u32Addr); // 送信元アドレス
			S_BE_DWORD(0xFFFFFFFF); // 送信先アドレス（指定無し）
			S_OCTET(pRx->u8Lqi);
			S_BE_WORD(u8paylen);
			memcpy(q, p, u8paylen); q += u8paylen;

			vSerOutput_ModbusAscii(&sSerStream, u8AppLogicalId, SERCMD_ID_SERCMD_EX_SIMPLE, au8buff2, q - au8buff2);
		} else {
			vSerOutput_ModbusAscii(&sSerStream, u8AppLogicalId, u8Cmd, p, p_end - p);
		}
	}
}

/** @ingroup FLASH
 * フラッシュ設定構造体をデフォルトに巻き戻します。
 * - ここでシステムのデフォルト値を決めています。
 *
 * @param p 構造体へのアドレス
 */
static void vConfig_SetDefaults(tsFlashApp *p) {
	p->u32appid = APP_ID;
	p->u32chmask = CHMASK;
	p->u8ch = CHANNEL;
	p->u8pow = 3;
	p->u8id = 0;
	p->u8role = E_APPCONF_ROLE_MAC_NODE;
	p->u8layer = 1;

	p->u16SleepDur_ms = (MODE4_SLEEP_DUR_ms);
	p->u16SleepDur_s = (MODE7_SLEEP_DUR_ms / 1000);
	p->u8Fps = 16;

	p->u32baud_safe = UART_BAUD_SAFE;
	p->u8parity = 0; // none

	p->u32Opt = 0;

	p->u8Crypt = 0;
	p->au8AesKey[0] = 0;

	p->u32HoldMask = 0;
	p->u16HoldDur_ms = 1000;
}

/** @ingroup FLASH
 * フラッシュ設定構造体を全て未設定状態に設定します。未設定状態は全て 0xFF と
 * なり、逆にいえば 0xFF, 0xFFFF といった設定値を格納できます。
 *
 * @param p 構造体へのアドレス
 */
static void vConfig_UnSetAll(tsFlashApp *p) {
	memset (p, 0xFF, sizeof(tsFlashApp));
}

/** @ingroup FLASH
 * フラッシュ(またはEEPROM)に保存し、モジュールをリセットする
 */
static void vConfig_SaveAndReset() {
	tsFlash sFlash = sAppData.sFlash;

	if (sAppData.sConfig_UnSaved.u32appid != 0xFFFFFFFF) {
		sFlash.sData.u32appid = sAppData.sConfig_UnSaved.u32appid;
	}
	if (sAppData.sConfig_UnSaved.u32chmask != 0xFFFFFFFF) {
		sFlash.sData.u32chmask = sAppData.sConfig_UnSaved.u32chmask;
	}
	if (sAppData.sConfig_UnSaved.u8id != 0xFF) {
		sFlash.sData.u8id = sAppData.sConfig_UnSaved.u8id;
	}
	if (sAppData.sConfig_UnSaved.u8ch != 0xFF) {
		sFlash.sData.u8ch = sAppData.sConfig_UnSaved.u8ch;
	}
	if (sAppData.sConfig_UnSaved.u8pow != 0xFF) {
		sFlash.sData.u8pow = sAppData.sConfig_UnSaved.u8pow;
	}
	if (sAppData.sConfig_UnSaved.u8layer != 0xFF) {
		sFlash.sData.u8layer = sAppData.sConfig_UnSaved.u8layer;
	}
	if (sAppData.sConfig_UnSaved.u8role != 0xFF) {
		sFlash.sData.u8role = sAppData.sConfig_UnSaved.u8role;
	}
	if (sAppData.sConfig_UnSaved.u16SleepDur_ms != 0xFFFF) {
		sFlash.sData.u16SleepDur_ms = sAppData.sConfig_UnSaved.u16SleepDur_ms;
	}
	if (sAppData.sConfig_UnSaved.u16SleepDur_s != 0xFFFF) {
		sFlash.sData.u16SleepDur_s = sAppData.sConfig_UnSaved.u16SleepDur_s;
	}
	if (sAppData.sConfig_UnSaved.u8Fps != 0xFF) {
		sFlash.sData.u8Fps = sAppData.sConfig_UnSaved.u8Fps;
	}
	if (sAppData.sConfig_UnSaved.u32baud_safe != 0xFFFFFFFF) {
		sFlash.sData.u32baud_safe = sAppData.sConfig_UnSaved.u32baud_safe;
	}
	if (sAppData.sConfig_UnSaved.u8parity != 0xFF) {
		sFlash.sData.u8parity = sAppData.sConfig_UnSaved.u8parity;
	}
	if (sAppData.sConfig_UnSaved.u32Opt != 0xFFFFFFFF) {
		sFlash.sData.u32Opt = sAppData.sConfig_UnSaved.u32Opt;
	}
	if (sAppData.sConfig_UnSaved.u8Crypt != 0xFF) {
		sFlash.sData.u8Crypt = sAppData.sConfig_UnSaved.u8Crypt;
	}
	{
		int i;
		for (i = 0; i < sizeof(sAppData.sConfig_UnSaved.au8AesKey); i++) {
			if (sAppData.sConfig_UnSaved.au8AesKey[i] != 0xFF) break;
		}
		if (i != sizeof(sAppData.sConfig_UnSaved.au8AesKey)) {
			memcpy(sFlash.sData.au8AesKey,
					sAppData.sConfig_UnSaved.au8AesKey,
					sizeof(sAppData.sConfig_UnSaved.au8AesKey));
		}
	}
	if (sAppData.sConfig_UnSaved.u32HoldMask != 0xFFFFFFFF) {
		sFlash.sData.u32HoldMask = sAppData.sConfig_UnSaved.u32HoldMask;
	}
	if (sAppData.sConfig_UnSaved.u16HoldDur_ms != 0xFFFF) {
		sFlash.sData.u16HoldDur_ms = sAppData.sConfig_UnSaved.u16HoldDur_ms;
	}

	sFlash.sData.u32appkey = APP_ID;
	sFlash.sData.u32ver = VERSION_U32;

	bool_t bRet = bFlash_Write(&sFlash, FLASH_SECTOR_NUMBER - 1, 0);
	V_PRINT("!INF FlashWrite %s"LB, bRet ? "Success" : "Failed");
	vConfig_UnSetAll(&sAppData.sConfig_UnSaved);
	vWait(100000);

	V_PRINT("!INF RESET SYSTEM...");
	vWait(1000000);
	vAHI_SwReset();
}

/** @ingroup MASTER
 * スリープ状態に遷移します。
 *
 * @param u32SleepDur_ms スリープ時間[ms]
 * @param bPeriodic TRUE:前回の起床時間から次のウェイクアップタイミングを計る
 * @param bDeep TRUE:RAM OFF スリープ
 */
static void vSleep(uint32 u32SleepDur_ms, bool_t bPeriodic, bool_t bDeep, bool_t bNoIoInt) {
	// IO 情報の保存
	memcpy(&sAppData.sIOData_reserve, &sAppData.sIOData_now, sizeof(tsIOData));

	// stop interrupt source, if interrupt source is still running.
	vAHI_DioInterruptEnable(0, u32_PORT_INPUT_MASK); // 割り込みの解除）

	// set UART Rx port as interrupt source
	vAHI_DioSetDirection(u32_PORT_INPUT_MASK, 0); // set as input

	(void)u32AHI_DioInterruptStatus(); // clear interrupt register

	if (!bNoIoInt) {
		vAHI_DioWakeEnable(u32_PORT_INPUT_MASK, 0); // also use as DIO WAKE SOURCE
		vAHI_DioWakeEdge(0, u32_PORT_INPUT_MASK); // 割り込みエッジ（立下りに設定）
		// vAHI_DioWakeEnable(0, u32_PORT_INPUT_MASK); // DISABLE DIO WAKE SOURCE
	}

	// wake up using wakeup timer as well.
	ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, u32SleepDur_ms, bPeriodic, bDeep); // PERIODIC RAM OFF SLEEP USING WK0
}


/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
