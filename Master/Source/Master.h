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

/** @file
 * アプリケーションのメイン処理
 *
 * @defgroup MASTER アプリケーションのメイン処理
 */

#ifndef  MASTER_H_INCLUDED
#define  MASTER_H_INCLUDED

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <stdlib.h>
#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "ToCoNet.h"
#include "flash.h"
#include "btnMgr.h"

#include "common.h"

/** @ingroup MASTER
 * ネットワークのモード列挙体 (ショートアドレス管理か中継ネットワーク層を使用したものか)
 */
typedef enum {
	E_NWKMODE_MAC_DIRECT,//!< ネットワークは構成せず、ショートアドレスでの通信を行う
	E_NWKMODE_LAYERTREE  //!< 中継ネットワークでの通信を行う(これは使用しない)
} teNwkMode;

/** @ingroup MASTER
 * IO の状態
 */
typedef struct {
	uint32 u32BtmBitmap; //!< (0xFFFFFFFF: 未確定)
	uint32 u32BtmUsed; //!< 利用対象ピンかどうか (0xFFFFFFFF: 未確定)
	uint32 u32BtmChanged; //!< (0xFFFFFFFF: 未確定)

	uint8 au8Input[MAX_IO]; //!< 入力ポート (0: Hi, 1: Lo, 0xFF: 未確定)
	uint8 au8Output[MAX_IO]; //!< 出力ポート (0: Hi, 1:Lo, 0xFF: 未確定)

	uint32 u32TxLastTick; //!< 最後に送った時刻
	uint32 u32RxLastTick; //!< 最後に受信した時刻

	int16 i16TxCbId; //!< 送信時のID
} tsIOData;

/** @ingroup MASTER
 * IO 設定要求
 */
typedef struct {
	uint8 u16IOports;          //!< 出力IOの状態 (1=Lo, 0=Hi)
	uint8 u16IOports_use_mask; //!< 設定を行うポートなら TRU
} tsIOSetReq;

/** @ingroup MASTER
 * アプリケーションの情報
 */
typedef struct {
	// ToCoNet
	uint32 u32ToCoNetVersion; //!< ToCoNet のバージョン番号を保持
	uint16 u16ToCoNetTickDelta_ms; //!< ToCoNet の Tick 周期 [ms]
	uint8 u8AppIdentifier; //!< AppID から自動決定

	// メインアプリケーション処理部
	void *prPrsEv; //!< vProcessEvCoreSlpまたはvProcessEvCorePwrなどの処理部へのポインタ

	// DEBUG
	uint8 u8DebugLevel; //!< デバッグ出力のレベル

	// Wakeup
	bool_t bWakeupByButton; //!< TRUE なら起床時に DI 割り込みにより起床した
	uint32 u32SleepDur; //!< スリープ間隔 [ms]

	// mode3 fps
	uint8 u8FpsBitMask; //!< mode=3 連続送信時の秒間送信タイミングを判定するためのビットマスク (64fps のカウンタと AND を取って判定)

	// Network mode
	teNwkMode eNwkMode; //!< ネットワークモデル(未使用：将来のための拡張用)
	uint8 u8AppLogicalId; //!< ネットワーク時の抽象アドレス 0:親機 1~:子機, 0xFF:通信しない

	// Network context
	tsToCoNet_Nwk_Context *pContextNwk; //!< ネットワークコンテキスト(未使用)
	tsToCoNet_NwkLyTr_Config sNwkLayerTreeConfig; //!< LayerTree の設定情報(未使用)

	// Flash Information
	tsFlash sFlash; //!< フラッシュからの読み込みデータ
	tsFlashApp sConfig_UnSaved; //!< フラッシュへの設定データ (0xFF, 0xFFFFFFFF は未設定)
	int8 bFlashLoaded; //!< フラッシュからの読み込みが正しく行われた場合は TRUE

	uint32 u32DIO_startup; //!< 電源投入時のIO状態

	// config mode
	uint8 u8Mode; //!< 動作モード(IO M1,M2,M3 から設定される)
	uint8 u8ChCfg; //!< チャネル設定(EI1,EI2 から設定される)
#ifdef JN514x
	bool_t bStrong; //!< Strong モジュールの判定
#endif

	// button manager
	tsBTM_Config sBTM_Config; //!< ボタン入力（連照により状態確定する）管理構造体
	PR_BTM_HANDLER pr_BTM_handler; //!< ボタン入力用のイベントハンドラ (TickTimer 起点で呼び出す)
	uint32 u32BTM_Tick_LastChange; //!< ボタン入力で最後に変化が有ったタイムスタンプ (操作の無効期間を作る様な場合に使用する)

	uint8 u8MaxIoCount; //!< ボタンの最大数 (MAX_IO と不一致の運用もある)
	uint16 au16HoldBtn[MAX_IO]; //!< ボタンの入力を一定時間維持する
	uint32 u32BtnMask_Special; //!< ボタンの入力に対する特殊設定に対応するマスク

	// latest state
	tsIOData sIOData_now; //!< 現時点での IO 情報
	tsIOData sIOData_reserve; //!< 保存された状態(0がデフォルトとする)
	uint8 u8IOFixState; //!< IOの読み取り確定ビット

	// Counter
	uint32 u32CtTimer0; //!< 64fps カウンタ。スリープ後も維持
	uint16 u16CtTimer0; //!< 64fps カウンタ。起動時に 0 クリアする
	uint16 u16CtRndCt; //!< 起動時の送信タイミングにランダムのブレを作る

	uint8 u8UartReqNum; //!< UART の要求番号

	uint16 u16TxFrame; //!< 送信フレーム数
	uint8 u8SerMsg_RequestNumber; //!< シリアルメッセージの要求番号
} tsAppData;

/****************************************************************************
 * フラッシュ設定情報
 ***************************************************************************/

#define FL_MASTER_u32(c) sAppData.sFlash.sData.u32##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u32(c) sAppData.sConfig_UnSaved.u32##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u32(c) (sAppData.sConfig_UnSaved.u32##c != 0xFFFFFFFF)  //!< 構造体要素アクセス用のマクロ @ingroup FLASH

#define FL_MASTER_u16(c) sAppData.sFlash.sData.u16##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u16(c) sAppData.sConfig_UnSaved.u16##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u16(c) (sAppData.sConfig_UnSaved.u16##c != 0xFFFF)  //!< 構造体要素アクセス用のマクロ @ingroup FLASH

#define FL_MASTER_u8(c) sAppData.sFlash.sData.u8##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_UNSAVE_u8(c) sAppData.sConfig_UnSaved.u8##c //!< 構造体要素アクセス用のマクロ @ingroup FLASH
#define FL_IS_MODIFIED_u8(c) (sAppData.sConfig_UnSaved.u8##c != 0xFF) //!< 構造体要素アクセス用のマクロ @ingroup FLASH

/** @ingroup FLASH
 * フラッシュ設定内容の列挙体
 */
enum {
	E_APPCONF_APPID,     //!< アプリケーションID
	E_APPCONF_CHMASK,    //!< チャネルマスク
	E_APPCONF_TX_POWER,  //!< TX 出力
	E_APPCONF_ID,        //!< 8bitのID(ネットワークアドレス)
	E_APPCONF_ROLE,      //!<
	E_APPCONF_LAYER ,    //!<
	E_APPCONF_SLEEP4,    //!< mode4 のスリープ期間設定
	E_APPCONF_SLEEP7,    //!< mode7 のスリープ期間設定
	E_APPCONF_FPS,       //!< 連続送信モードの秒あたりの送信数
	E_APPCONF_PWM_HZ,    //!< PWM の周波数
	E_APPCONF_SYS_HZ,    //!<
	E_APPCONF_OPT,       //!< DIOの入力方法に関する設定
	E_APPCONF_BAUD_SAFE, //!< BPS ピンをGにしたときのボーレート
	E_APPCONF_BAUD_PARITY, //!< BPS ピンをGにしたときのパリティ設定 (0:None, 1:Odd, 2:Even)
	E_APPCONF_CRYPT_MODE,  //!< 暗号化モード
	E_APPCONF_CRYPT_KEY,   //!< 暗号化鍵
	E_APPCONF_HOLD_MASK,   //!< Lo 維持の対象ポート
	E_APPCONF_HOLD_DUR,    //!< Lo 維持の期間
	E_APPCONF_TEST
};

/** @ingroup FLASH
 * フラッシュ設定で ROLE に対する要素名の列挙体
 * (未使用、将来のための拡張のための定義)
 */
enum {
	E_APPCONF_ROLE_MAC_NODE = 0,  //!< MAC直接のノード（親子関係は無し）
	E_APPCONF_ROLE_NWK_MASK = 0x10, //!< NETWORKモードマスク
	E_APPCONF_ROLE_PARENT,          //!< NETWORKモードの親
	E_APPCONF_ROLE_ROUTER,        //!< NETWORKモードの子
	E_APPCONF_ROLE_ENDDEVICE,     //!< NETWORKモードの子（未使用、スリープ対応）
	E_APPCONF_ROLE_SILENT = 0x7F, //!< 何もしない（設定のみ)
};

#define E_APPCONF_OPT_LOW_LATENCY_INPUT 0x0001UL //!< Hi>Lo を検知後直ぐに送信する。 @ingroup FLASH
#define E_APPCONF_OPT_LOW_LATENCY_INPUT_SLEEP_TX_BY_INT 0x0002UL //!< スリープを H>L 検出した場合に、割り込み要因のポートのみ送信する @ingroup FLASH
#define E_APPCONF_OPT_ON_PRESS_TRANSMIT 0x0100UL //!< 押し下げ時のみ送信する特殊動作モード。 @ingroup FLASH

#define IS_APPCONF_OPT_LOW_LATENCY_INPUT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT) //!< E_APPCONF_OPT_LOW_LATENCY_INPUT 判定 @ingroup FLASH
#define IS_APPCONF_OPT_LOW_LATENCY_INPUT_SLEEP_TX_BY_INT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_LOW_LATENCY_INPUT_SLEEP_TX_BY_INT) //!< E_APPCONF_OPT_LOW_LATENCY_INPUT_SLEEP_TX_BY_INT 判定 @ingroup FLASH
#define IS_APPCONF_OPT_ON_PRESS_TRANSMIT() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_ON_PRESS_TRANSMIT) //!< E_APPCONF_OPT_ON_PRESS_TRANSMIT判定 @ingroup FLASH

#define E_APPCONF_OPT_ACK_MODE 0x0010 //!< ACK付き通信を行う @ingroup FLASH
#define IS_APPCONF_OPT_ACK_MODE() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_ACK_MODE) //!< E_APPCONF_OPT_ACK_MODE判定 @ingroup FLASH

#define E_APPCONF_OPT_NO_REGULAR_TX 0x0020 //!< REGULAR 通信しない @ingroup FLASH
#define IS_APPCONF_OPT_NO_REGULAR_TX() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_NO_REGULAR_TX) //!< E_APPCONF_OPT_NO_REGULAR_TX判定 @ingroup FLASH

#define E_APPCONF_OPT_CHILD_RECV_OTHER_NODES 0x10000 //!< 子機通常モードで受信を可能とする @ingroup FLASH
#define IS_APPCONF_OPT_CHILD_RECV_OTHER_NODES() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_CHILD_RECV_OTHER_NODES) //!< E_APPCONF_OPT_CHILD_RECV_OTHER_NODES判定 @ingroup FLASH

#define E_APPCONF_OPT_CHILD_RECV_NO_IO_DATA 0x20000 //!< 子機通常モードで受信を可能とする @ingroup FLASH
#define IS_APPCONF_OPT_CHILD_RECV_NO_IO_DATA() (sAppData.sFlash.sData.u32Opt & E_APPCONF_OPT_CHILD_RECV_NO_IO_DATA) //!< E_APPCONF_OPT_CHILD_RECV_NO_IO_DATA判定 @ingroup FLASH

/** サイレントモードの判定マクロ  @ingroup FLASH */
#define IS_APPCONF_ROLE_SILENT_MODE() (sAppData.sFlash.sData.u8role == E_APPCONF_ROLE_SILENT)

/** AES 利用のマクロ判定  @ingroup FLASH */
#define IS_CRYPT_MODE() (sAppData.sFlash.sData.u8Crypt)

/****************************************************************************
 * 内部処理
 ***************************************************************************/
#define E_IO_FIX_STATE_NOT_READY 0x0
#define E_IO_FIX_STATE_READY 0x1

#endif  /* MASTER_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
