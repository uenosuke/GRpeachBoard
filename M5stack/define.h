#ifndef DEFINE
#define DEFINE

struct coords{
  double x;
  double y;
  double z;
};

#define INT_TIME ( 20 )

#define SPRITE_W ( 320 )
#define SPRITE_H ( 200 )

#define SPRITE_POS_X ( 0 )
#define SPRITE_POS_Y ( 20 )

#define STATE_LPMS_ENABLE 0x01
#define STATE_SD_INIT     0x02
#define STATE_SD_WRITE    0x04
#define STATE_ZONE        0x08

#define MASK_BUTTON_X  0x0001
#define MASK_BUTTON_Y  0x0002
#define MASK_BUTTON_A  0x0004
#define MASK_BUTTON_B  0x0008

#define MASK_BUTTON_L1     0x0010
#define MASK_BUTTON_R1     0x0020
#define MASK_BUTTON_L2     0x0040
#define MASK_BUTTON_R2     0x0080

//#define MASK_BUTTON_PS     0x0100 // PS4のときはPSボタン
//#define MASK_BUTTON_PAD    0x0200 // PS4のときはパッド
#define MASK_BUTTON_JOY_L   0x0100
#define MASK_BUTTON_JOY_R   0x0200
#define MASK_BUTTON_BACK    0x0400
#define MASK_BUTTON_START   0x0800

#define MASK_BUTTON_UP     0x1000
#define MASK_BUTTON_RIGHT  0x2000
#define MASK_BUTTON_DOWN   0x4000
#define MASK_BUTTON_LEFT   0x8000

#endif
