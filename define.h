#ifndef DEFINE_h
#define DEFINE_h

#include "Arduino.h"

struct coords{
    double x;
    double y;
    double z;
};

// Platformの種類を設定
#define PLATFORM_MECHANUM      ( 0 )
#define PLATFORM_OMNI4WHEEL    ( 1 )
#define PLATFORM_OMNI3WHEEL    ( 2 )
#define PLATFORM_DUALWHEEL     ( 3 )

#define DRIVE_UNIT  ( PLATFORM_DUALWHEEL )

// ManualControlクラス用の定義
#define JOY_DEADBAND    ( 5 )
#define JOY_MAXVEL      ( 1.0 )
#define JOY_MAXANGVEL   ( 2.5 )


#define SERIAL_LPMSME1  Serial1
#define SERIAL_ROBOCLAW Serial4
#define SERIAL_LEONARDO Serial5
#define SERIAL_LCD      Serial6
#define SERIAL_XBEE     Serial7

#define PIN_XBEERESET 66

// スイッチやLEDのピン設定
#define PIN_DIP1 25
#define PIN_DIP2 24
#define PIN_DIP3 69
#define PIN_DIP4 70

#define PIN_SW_UP    32
#define PIN_SW_LEFT  33
#define PIN_SW_RIGHT 31
#define PIN_SW_DOWN  30

#define PIN_SW_WHITE  29
#define PIN_SW_YELLOW 28

#define PIN_ENC_A  26
#define PIN_ENC_B  27

#define PIN_LED_1 20
#define PIN_LED_2 36
#define PIN_LED_3 37
#define PIN_LED_4 38
#define PIN_LED_ENC 40

#define PIN_CSB 10

// Lernardo からのコントローラ用データのマスクデータ
#define BUTTON_X  0x0001
#define BUTTON_Y  0x0002
#define BUTTON_A  0x0004
#define BUTTON_B  0x0008

#define BUTTON_L1     0x0010
#define BUTTON_R1     0x0020
#define BUTTON_L2     0x0040
#define BUTTON_R2     0x0080

#define BUTTON_JOY_L   0x0100
#define BUTTON_JOY_R   0x0200
#define BUTTON_BACK    0x0400
#define BUTTON_START   0x0800

#define BUTTON_UP     0x1000
#define BUTTON_RIGHT  0x2000
#define BUTTON_DOWN   0x4000
#define BUTTON_LEFT   0x8000

// 制御周期
#define INT_TIME			( 0.01 )//( 0.001 )

// フェーズ管理
//#define STATE1      ( 10 )// スタートからゲルゲ受け渡しまで(0から数えて)
//#define STATE1_1    ( 7 )// ベジエTANGENTモード
//#define STATE1_2    ( 8 )// ベジエCOMMANDモード
//#define STATE1_3    ( 9 )// ベジエCOMMANDモード(フェーズの変更は収束判定ではなくリミットスイッチで)
//#define STATE1_4    ( 10 )// 使うか分からないけど
//#define STATE2      ( 14 )// ゲルゲ受け渡し後からシャガイ取得まで
//#define STATE3      ( 15 )// シャガイ取得後からスローイングゾーン待機まで
//#define STATE4      ( 17 )// 投擲位置まで移動
//#define STATE5      ( 20 )//( 19 )// 2個目のシャガイまで
//#define STATE6      ( 22 )//( 21 )// シャガイ取得後からスローイングゾーン待機まで
//#define STATE7      ( 25 )//( 23 )// 3個目のシャガイまで
//#define STATE8      ( 27 )//( 25 )// シャガイ取得後からスローイングゾーン待機まで

//#define STATE_ALL   ( STATE1 + STATE2 + STATE3 + STATE4 )

#endif
