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

#ifndef COMMON_H_
#define COMMON_H_

#include <serial.h>
#include <fprintf.h>

#include "flash.h"
#include "serialInputMgr.h"

#include "config.h"
#include "common.h"

#include "Version.h"

/** @ingroup MASTER
 * 最大のIO数
 */
#define MAX_IO 12

/** @ingroup MASTER
 * 使用する無線チャネル数の最大値 (複数設定すると Channel Agility を利用する)
 */
#define MAX_CHANNELS 3

/*
 * Version 番号
 */
#define VERSION_U32 ((VERSION_MAIN << 16) | (VERSION_SUB << 8) | (VERSION_VAR))

/*
 * IOポートの定義
 */
#if defined(JN516x)
#ifdef USE_DEV_KIT_002_L
// 評価キットの定義
#warning "IO CONF IS FOR JN516X DEVKIT!"
#define PORT_OUT1 17 // KIT の LED
#define PORT_OUT2 13
#define PORT_OUT3 12
#define PORT_OUT4 11

#define PORT_INPUT1 2 // KIT の SW
#define PORT_INPUT2 3
#define PORT_INPUT3 10
#define PORT_INPUT4 9

#define PORT_CONF1 0
#define PORT_CONF2 1
#define PORT_CONF3 8

#define PORT_I2C_CLK 14
#define PORT_I2C_DAT 15

#define PORT_EI1 4
#define PORT_EI2 5
#define PORT_EO1 18
#define PORT_EO2 19

#define PORT_BAUD 16
#else
// TWE-Lite DIP (TWELITE の標準構成)
#define PORT_OUT1 18
#define PORT_OUT2 19
#define PORT_OUT3 4
#define PORT_OUT4 9

#define PORT_INPUT1 12
#define PORT_INPUT2 13
#define PORT_INPUT3 11
#define PORT_INPUT4 16

#define PORT_CONF1 10
#define PORT_CONF2 2
#define PORT_CONF3 3

#define PORT_BAUD 17

#define PORT_I2C_CLK 14
#define PORT_I2C_DAT 15

#define PORT_EI1 0 // AI2
#define PORT_EI2 1 // AI4
#define PORT_EO1 5 // PWM1
#define PORT_EO2 8 // PWM4

#endif
#elif defined(JN514x)
#define PORT_OUT1 16 // KIT LED1
#define PORT_OUT2 17 // KIT LED2
#define PORT_OUT3 18 // KIT LED3
#define PORT_OUT4 19 // KIT LED4

#define PORT_INPUT1 9 // KIT SW1
#define PORT_INPUT2 10 // KIT SW2
#define PORT_INPUT3 11 // KIT SW3
#define PORT_INPUT4 20 // KIT SW4

#define PORT_CONF1 0
#define PORT_CONF2 1
#define PORT_CONF3 8

#define PORT_BAUD 12

#define PORT_I2C_CLK 14
#define PORT_I2C_DAT 15

#define PORT_EI1 4
#define PORT_EI2 5
#define PORT_EO1 13
#define PORT_EO2 3

#warning "PWM1->DIO13, PWM2->DIO11"
#endif

#define PORT_OUT_MASK ((1UL << PORT_OUT1) | (1UL << PORT_OUT2) | (1UL << PORT_OUT3) | (1UL << PORT_OUT4) | \
					   (1UL << PORT_INPUT1) | (1UL << PORT_INPUT2) | (1UL << PORT_INPUT3) | (1UL << PORT_INPUT4) | \
					   (1UL << PORT_I2C_CLK) | (1UL << PORT_I2C_DAT) | (1UL << PORT_EO1) | (1UL << PORT_EO2))
#define PORT_INPUT_MASK PORT_OUT_MASK

// マップテーブル
extern const uint8 au8PortTbl_DOut[MAX_IO]; //!< IO番号(出力)のテーブル
extern const uint8 au8PortTbl_DIn[MAX_IO]; //!< IO番号(入力)のテーブル

/**
 * PORT_CONF1 ～ 3 による定義
 */
typedef enum {
	E_IO_MODE_CHILD = 0,          //!< E_IO_MODE_CHILD
	E_IO_MODE_PARNET,             //!< E_IO_MODE_PARNET
	E_IO_MODE_ROUTER,             //!< E_IO_MODE_ROUTER
	E_IO_MODE_CHILD_CONT_TX,      //!< E_IO_MODE_CHILD_CONT_TX
	E_IO_MODE_CHILD_SLP_1SEC,     //!< E_IO_MODE_CHILD_SLP_1SEC
	E_IO_MODE_CHILD_SLP_10SEC = 7,//!< E_IO_MODE_CHILD_SLP_10SEC
} tePortConf2Mode;
extern const uint8 au8IoModeTbl_To_LogicalID[8]; //!< tePortConf2Mode から論理アドレスへの変換

#define LOGICAL_ID_PARENT (0)
#define LOGICAL_ID_CHILDREN (120)
#define LOGICAL_ID_REPEATER (254)
#define LOGICAL_ID_BROADCAST (255)

#define IS_LOGICAL_ID_CHILD(s) (s>0 && s<128) //!< 論理アドレスが子機の場合
#define IS_LOGICAL_ID_PARENT(s) (s == 0) //!< 論理アドレスが親機の場合
#define IS_LOGICAL_ID_REPEATER(s) (s == 254) //!< 論理アドレスがリピータの場合

/**
 *
 */
extern const uint32 au32ChMask_Preset[];

/*
 * シリアルコマンドの定義
 */
#define APP_PROTOCOL_VERSION 0x01 //!< プロトコルバージョン

#define SERCMD_ADDR_TO_MODULE 0xDB //!< Device -> 無線モジュール
#define SERCMD_ADDR_TO_PARENT 0x00 //!< Device
#define SERCMD_ADDR_FR_MODULE 0xDB //!< 無線モジュール -> Device
#define SERCMD_ADDR_FR_PARENT 0x0 //!< ...-> (無線送信) -> Device

#define SERCMD_ADDR_CONV_TO_SHORT_ADDR(c) (c + 0x100) //!< 0..255 の論理番号をショートアドレスに変換する
#define SERCMD_ADDR_CONV_FR_SHORT_ADDR(c) (c - 0x100) //!< ショートアドレスを論理番号に変換する

#define SERCMD_ID_ACK 0xF0

#define SERCMD_ID_REQUEST_IO_DATA 0x80
#define SERCMD_ID_INFORM_IO_DATA 0x81

#define SERCMD_ID_I2C_COMMAND 0x88
#define SERCMD_ID_I2C_COMMAND_RESP 0x89

#define SERCMD_ID_GET_MODULE_ADDRESS 0x90
#define SERCMD_ID_INFORM_MODULE_ADDRESS 0x91
#define SERCMD_ID_GET_NETWORK_CONFIG 0x92
#define SERCMD_ID_INFORM_NETWORK_CONFIG 0x93
#define SERCMD_ID_SET_NETWORK_CONFIG 0x94

#define SERCMD_ID_SERCMD_EX_SIMPLE 0xA4

// Packet CMD IDs
#define TOCONET_PACKET_CMD_APP_USER_IO_DATA (TOCONET_PACKET_CMD_APP_USER+0) //!< IO の入出力データ
#define TOCONET_PACKET_CMD_APP_USER_IO_DATA_EXT (TOCONET_PACKET_CMD_APP_USER+1) //!< IO の入出力データ
#define TOCONET_PACKET_CMD_APP_USER_SERIAL_MSG (TOCONET_PACKET_CMD_APP_USER+2) //!< IO の入出力データ

/* Modbus ASCII output functions */
void vModbOut_AckNack(tsFILE *pSer, bool_t bAck);
void vModbOut_MySerial(tsFILE *pSer);

bool_t bModbIn_Config(uint8 *p,  tsFlashApp *pConfig);
void vModbOut_Config(tsFILE *pSer, tsFlashApp *pConfig);

/*
 * 出力マクロ
 */
#define V_PRINT(...) if(sSerCmdIn.bverbose) vfPrintf(&sSerStream,__VA_ARGS__) //!< VERBOSE モード時の printf 出力
#define S_PRINT(...) if(!sSerCmdIn.bverbose) vfPrintf(&sSerStream,__VA_ARGS__) //!< 非VERBOSE モード時の printf 出力
#define V_PUTCHAR(c) if(sSerCmdIn.bverbose) sSerStream.bPutChar(sSerStream.u8Device, c)  //!< VERBOSE モード時の putchar 出力
#define S_PUTCHAR(c) if(!sSerCmdIn.bverbose) sSerStream.bPutChar(sSerStream.u8Device, c) //!< 非VERBOSE モード時の putchar 出力
#ifdef DEBUG_OUTPUT
#define DBGOUT(lv, ...) if(sAppData.u8DebugLevel >= lv) vfPrintf(&sSerStream, __VA_ARGS__) //!< デバッグ出力
#else
#define DBGOUT(lv, ...) //!< デバッグ出力
#endif

#endif /* COMMON_H_ */
