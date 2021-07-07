#include "DisplayControl.h"

#define COLOR_PINK 0xfb56 // ピンクの設定
#define COLOR_GREEN 0x1c43 // 良い感じの緑色の設定
#define COLOR_NAVY 0x0011 // 良い感じの濃い青の設定

// 共通するグローバル変数をextern
extern TFT_eSprite lcd; // main で宣言されている
extern unsigned int robotState;
extern unsigned int buttonState;
extern uint8_t LJoyX, LJoyY, RJoyX, RJoyY;
extern coords gPosi, refVel;
extern int dispNum;
extern String disp_msgs;

// 過去のメッセージを保持するための変数
String past_disp_msgs1 = "";
String past_disp_msgs2 = "";

bool firstDraw = false; // 初回描画を判断するための変数

// ディスプレイの初期化関数
void initDisplay()
{
  dispRobotState(); // ロボット状態表示部の初期化
  dispConnectionState(false); // 接続状態表示部の初期化

  initSpriteArea(); // それ以外の外部分のエリア(Spriteエリア)の初期化
}

// Spriteエリアの初期化
void initSpriteArea()
{
  if (dispNum == 0)
  {
    M5.Lcd.fillRect(0, 220, 320, 20, TFT_BLACK);
    M5.Lcd.fillTriangle(52, 220, 82, 220, 67, 235, COLOR_PINK);
    M5.Lcd.drawTriangle(145, 220, 175, 220, 160, 235, TFT_LIGHTGREY);
    M5.Lcd.drawTriangle(236, 220, 268, 220, 252, 235, TFT_LIGHTGREY);

    // Sprash領域
    lcd.fillSprite(TFT_BLACK);
    // 指令速度表示部
    lcd.setTextSize(1);
    lcd.setTextColor(COLOR_PINK);
    lcd.setCursor(3, 30);
    lcd.print("refVx:");

    lcd.setCursor(109, 30);
    lcd.print("refVy:");

    lcd.setCursor(217, 30);
    lcd.print("refVz:");

    // ロボット位置表示部
    lcd.setTextColor(TFT_WHITE);
    lcd.setCursor(25, 60);
    lcd.print("gPosiX");

    lcd.setCursor(131, 60);
    lcd.print("gPosiY");

    lcd.setCursor(239, 60);
    lcd.print("gPosiZ");

    lcd.drawLine(10, 40, 310, 40, TFT_WHITE);

    lcd.fillRect(0, 135, 320, 65, TFT_BLACK);

    lcd.setTextColor(TFT_DARKGREY);
    lcd.setTextSize(1);

    if (past_disp_msgs2.length() == 0) {
      if (past_disp_msgs1.length() == 0) {
        if (disp_msgs.length() != 0) {
          lcd.setCursor(15, 147);
          lcd.print(">> " + disp_msgs);
        }
      } else {
        lcd.setCursor(15, 147);
        lcd.print(">> " + past_disp_msgs1);
        lcd.setCursor(15, 167);
        lcd.print(">> " + disp_msgs);
      }
    } else {
      lcd.setCursor(15, 147);
      lcd.print(">> " + past_disp_msgs2);
      lcd.setCursor(15, 167);
      lcd.print(">> " + past_disp_msgs1);
      lcd.setCursor(15, 187);
      lcd.print(">> " + disp_msgs);
    }
  }
  else if (dispNum == 1)
  {
    M5.Lcd.fillRect(0, 220, 320, 20, TFT_BLACK);
    M5.Lcd.drawTriangle(52, 220, 82, 220, 67, 235, TFT_LIGHTGREY);
    M5.Lcd.fillTriangle(145, 220, 175, 220, 160, 235, COLOR_PINK);
    M5.Lcd.drawTriangle(236, 220, 268, 220, 252, 235, TFT_LIGHTGREY);
    ets_delay_us(10);

    // Sprash領域
    lcd.fillSprite(TFT_BLACK);
    // ロボット位置表示部
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_WHITE);
    lcd.setCursor(5, 50);
    lcd.print("gPosiX");

    lcd.setCursor(90, 110);
    lcd.print("gPosiY");

    lcd.setCursor(5, 170);
    lcd.print("gPosiZ");
  }
  else if (dispNum = 2)
  {
    M5.Lcd.fillRect(0, 220, 320, 20, TFT_BLACK);
    M5.Lcd.drawTriangle(52, 220, 82, 220, 67, 235, TFT_LIGHTGREY);
    M5.Lcd.drawTriangle(145, 220, 175, 220, 160, 235, TFT_LIGHTGREY);
    M5.Lcd.fillTriangle(236, 220, 268, 220, 252, 235, COLOR_PINK);

    lcd.fillSprite(TFT_BLACK);

    ets_delay_us(10);

    lcd.setTextSize(1);
    lcd.setTextColor(TFT_WHITE);
    lcd.setCursor(5, 15);
    lcd.print("Controller state");
  }

  firstDraw = true;

  lcd.pushSprite(SPRITE_POS_X, SPRITE_POS_Y);
  ets_delay_us(10);
}

// 表示内容を更新するときの処理(必要な部分だけ更新するようにしている)
void refreshDisplay()
{
  static int state = 0;
  static int count = 0;
  static String current_disp_msgs = "";

  if (dispNum == 0)
  {
    printVelData();
    printPosiData();

    if (disp_msgs.length() != 0) {
      if (disp_msgs.compareTo(current_disp_msgs) != 0) {
        lcd.fillRect(0, 135, 320, 65, TFT_BLACK);

        lcd.setTextColor(TFT_DARKGREY);
        lcd.setTextSize(1);

        if (past_disp_msgs1.length() == 0) {
          if (current_disp_msgs.length() == 0) {
            lcd.setCursor(15, 147);
            lcd.print(">> " + disp_msgs);
            current_disp_msgs = disp_msgs;
          } else {
            lcd.setCursor(15, 147);
            lcd.print(">> " + current_disp_msgs);
            lcd.setCursor(15, 167);
            lcd.print(">> " + disp_msgs);
            past_disp_msgs1 = current_disp_msgs;
            current_disp_msgs = disp_msgs;
          }
        } else {
          lcd.setCursor(15, 147);
          lcd.print(">> " + past_disp_msgs1);
          lcd.setCursor(15, 167);
          lcd.print(">> " + current_disp_msgs);
          lcd.setCursor(15, 187);
          lcd.print(">> " + disp_msgs);
          past_disp_msgs2 = past_disp_msgs1;
          past_disp_msgs1 = current_disp_msgs;
          current_disp_msgs = disp_msgs;
        }
      }
    }
    
    // 右下で点滅させているやつ
    if (state)
    {
      count -= 0x0C;
      if (count <= 0)
        state = ~state;
    }
    else
    {
      count += 0x0C;
      if (count >= 0x3C)
        state = ~state;
    }
    lcd.fillEllipse(305, 165, 12, 12, count << 5);
  }
  else if (dispNum == 1)
  {
    printPosiData();

    // 右下で点滅させているやつ
    if (state)
    {
      count -= 0x0C;
      if (count <= 0)
        state = ~state;
    }
    else
    {
      count += 0x0C;
      if (count >= 0x3C)
        state = ~state;
    }
    lcd.fillEllipse(305, 165, 12, 12, count << 5);
  }
  else if (dispNum == 2)
  {
    // ジョイスティック用
    lcd.fillRect(65, 20, 110, 110, TFT_BLACK);
    lcd.fillRect(155, 20, 110, 110, TFT_BLACK);

    //lcd.drawRect(75, 40, 80, 80, TFT_WHITE);
    lcd.drawLine(75, 80, 155, 80, TFT_WHITE);
    lcd.drawLine(115, 40, 115, 120, TFT_WHITE);
    //lcd.drawEllipse(110, 80, 40, 40, TFT_WHITE);
    int posDotX = 120 - LJoyX * 1.2698;
    int posDotY = 155 - LJoyY * 1.2698;
    lcd.fillEllipse(posDotY, posDotX, 5, 5, COLOR_PINK);

    //lcd.drawRect(165, 40, 80, 80, TFT_DARKGREY);
    lcd.drawLine(165, 80, 245, 80, TFT_WHITE);
    lcd.drawLine(205, 40, 205, 120, TFT_WHITE);
    //lcd.drawEllipse(210, 80, 40, 40, TFT_WHITE);
    posDotX = 120 - RJoyX * 1.2698;
    posDotY = 245 - RJoyY * 1.2698;
    lcd.fillEllipse(posDotY, posDotX, 5, 5, COLOR_PINK);

    // 右側ボタン用
    lcd.fillEllipse(270, 120, 10, 10, TFT_BLACK);
    if (buttonState & MASK_BUTTON_Y) lcd.fillEllipse(270, 120, 10, 10, COLOR_PINK);
    else lcd.drawEllipse(270, 120, 10, 10, TFT_WHITE);

    lcd.fillEllipse(270, 170, 10, 10, TFT_BLACK);
    if (buttonState & MASK_BUTTON_A) lcd.fillEllipse(270, 170, 10, 10, COLOR_PINK);
    else lcd.drawEllipse(270, 170, 10, 10, TFT_WHITE);

    lcd.fillEllipse(245, 145, 10, 10, TFT_BLACK);
    if (buttonState & MASK_BUTTON_X) lcd.fillEllipse(245, 145, 10, 10, COLOR_PINK);
    else lcd.drawEllipse(245, 145, 10, 10, TFT_WHITE);

    lcd.fillEllipse(295, 145, 10, 10, TFT_BLACK);
    if (buttonState & MASK_BUTTON_B) lcd.fillEllipse(295, 145, 10, 10, COLOR_PINK);
    else lcd.drawEllipse(295, 145, 10, 10, TFT_WHITE);

    // 左側ボタン用
    lcd.fillRect(40, 110, 20, 20, TFT_BLACK);
    if (buttonState & MASK_BUTTON_UP) lcd.fillRect(40, 110, 20, 20, COLOR_PINK);
    else lcd.drawRect(40, 110, 20, 20, TFT_WHITE);

    lcd.fillRect(40, 160, 20, 20, TFT_BLACK);
    if (buttonState & MASK_BUTTON_DOWN) lcd.fillRect(40, 160, 20, 20, COLOR_PINK);
    else lcd.drawRect(40, 160, 20, 20, TFT_WHITE);

    lcd.fillRect(15, 135, 20, 20, TFT_BLACK);
    if (buttonState & MASK_BUTTON_LEFT) lcd.fillRect(15, 135, 20, 20, COLOR_PINK);
    else lcd.drawRect(15, 135, 20, 20, TFT_WHITE);

    lcd.fillRect(65, 135, 20, 20, TFT_BLACK);
    if (buttonState & MASK_BUTTON_RIGHT) lcd.fillRect(65, 135, 20, 20, COLOR_PINK);
    else lcd.drawRect(65, 135, 20, 20, TFT_WHITE);

    // 右上ボタン用
    lcd.fillRect(275, 55, 30, 20, TFT_BLACK);
    if (buttonState & MASK_BUTTON_R1) lcd.fillRect(275, 55, 30, 20, COLOR_PINK);
    else lcd.drawRect(275, 55, 30, 20, TFT_WHITE);

    lcd.fillRect(255, 30, 30, 20, TFT_BLACK);
    if (buttonState & MASK_BUTTON_R2) lcd.fillRect(255, 30, 30, 20, COLOR_PINK);
    else lcd.drawRect(255, 30, 30, 20, TFT_WHITE);

    // 左上ボタン用
    lcd.fillRect(15, 55, 30, 20, TFT_BLACK);
    if (buttonState & MASK_BUTTON_L1) lcd.fillRect(15, 55, 30, 20, COLOR_PINK);
    else lcd.drawRect(15, 55, 30, 20, TFT_WHITE);

    lcd.fillRect(35, 30, 30, 20, TFT_BLACK);
    if (buttonState & MASK_BUTTON_L2) lcd.fillRect(35, 30, 30, 20, COLOR_PINK);
    else lcd.drawRect(35, 30, 30, 20, TFT_WHITE);

    // 中央ボタン用
    lcd.fillRect(120, 145, 20, 15, TFT_BLACK);
    if (buttonState & MASK_BUTTON_BACK) lcd.fillRect(120, 145, 20, 15, COLOR_PINK);
    else lcd.drawRect(120, 145, 20, 15, TFT_WHITE);

    lcd.fillRect(120, 165, 20, 15, TFT_BLACK);
    if (buttonState & MASK_BUTTON_JOY_L) lcd.fillRect(120, 165, 20, 15, COLOR_PINK);
    else lcd.drawRect(120, 165, 20, 15, TFT_WHITE);

    lcd.fillRect(180, 145, 20, 15, TFT_BLACK);
    if (buttonState & MASK_BUTTON_START) lcd.fillRect(180, 145, 20, 15, COLOR_PINK);
    else lcd.drawRect(180, 145, 20, 15, TFT_WHITE);

    lcd.fillRect(180, 165, 20, 15, TFT_BLACK);
    if (buttonState & MASK_BUTTON_JOY_R) lcd.fillRect(180, 165, 20, 15, COLOR_PINK);
    else lcd.drawRect(180, 165, 20, 15, TFT_WHITE);

    ets_delay_us(10);

    // 右上で点滅させているやつ
    if (state)
    {
      count -= 0x0C;
      if (count <= 0)
        state = ~state;
    }
    else
    {
      count += 0x0C;
      if (count >= 0x3C)
        state = ~state;
    }
    ets_delay_us(10);
    lcd.fillEllipse(295, 17, 10, 10, count << 5);
  }

  lcd.pushSprite(SPRITE_POS_X, SPRITE_POS_Y);
  ets_delay_us(10);
}

// 速度の書き換え用関数
void printVelData()
{
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(1);

  lcd.setTextColor(TFT_WHITE);
  lcd.fillRect(52, 15, 50, 20, TFT_BLACK);
  lcd.setCursor(55, 30);
  lcd.printf("%5.2lf", gPosi.x);

  lcd.fillRect(158, 15, 50, 20, TFT_BLACK);
  lcd.setCursor(161, 30);
  lcd.printf("%5.2lf", gPosi.y);

  lcd.fillRect(266, 15, 50, 20, TFT_BLACK);
  lcd.setCursor(269, 30);
  lcd.printf("%5.2lf", gPosi.z);
}

// 座標データの書き換え用関数
void printPosiData()
{
  if (dispNum == 0) {
    lcd.setTextColor(TFT_BLACK);
    lcd.setTextSize(2);

    lcd.fillRoundRect(5, 68, 100, 55, 5, COLOR_PINK);
    lcd.setCursor(15, 106);
    lcd.printf("%5.2lf", refVel.x);

    lcd.fillRoundRect(110, 68, 100, 55, 5, COLOR_PINK);
    lcd.setCursor(120, 106);
    lcd.printf("%5.2lf", refVel.y);

    lcd.fillRoundRect(215, 68, 100, 55, 5, COLOR_PINK);
    lcd.setCursor(225, 106);
    lcd.printf("%5.2lf", refVel.z);
  }
  else if (dispNum == 1) {
    lcd.setTextColor(TFT_BLACK);
    lcd.setTextSize(3);

    lcd.fillRoundRect(75, 5, 150, 55, 5, COLOR_PINK);
    lcd.setCursor(90, 50);
    lcd.printf("%5.2lf", refVel.x);

    lcd.fillRoundRect(160, 65, 150, 55, 5, COLOR_PINK);
    lcd.setCursor(175, 110);
    lcd.printf("%5.2lf", refVel.y);

    lcd.fillRoundRect(75, 125, 150, 55, 5, COLOR_PINK);
    lcd.setCursor(90, 170);
    lcd.printf("%5.2lf", refVel.z);
  }
}

// 接続状態更新用関数
void dispConnectionState(bool state)
{
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_WHITE);

  if (state)
  {
    M5.Lcd.fillRect(245, 0, 75, 20, COLOR_GREEN); // 緑の塗りつぶし
    
    M5.Lcd.fillRect(272, 2, 10, 15, TFT_WHITE);
    M5.Lcd.fillEllipse(272, 9, 7, 7, TFT_WHITE);
    M5.Lcd.fillRect(257, 7, 10, 4, TFT_WHITE);

    M5.Lcd.fillRect(284, 2, 10, 15, TFT_WHITE);
    M5.Lcd.fillEllipse(294, 9, 7, 7, TFT_WHITE);
    M5.Lcd.fillRect(298, 7, 10, 4, TFT_WHITE);
  }
  else
  {
    M5.Lcd.fillRect(245, 0, 75, 20, TFT_DARKGREY);

    M5.Lcd.fillRect(278, 5, 5, 3, TFT_WHITE);
    M5.Lcd.fillRect(278, 11, 5, 3, TFT_WHITE);
    M5.Lcd.fillRect(268, 2, 10, 15, TFT_WHITE);
    M5.Lcd.fillEllipse(269, 9, 7, 7, TFT_WHITE);
    M5.Lcd.fillRect(253, 7, 10, 4, TFT_WHITE);

    M5.Lcd.fillRect(288, 2, 10, 15, TFT_WHITE);
    M5.Lcd.fillEllipse(298, 9, 7, 7, TFT_WHITE);
    M5.Lcd.fillRect(303, 7, 10, 4, TFT_WHITE);
  }
}

// ロボット状態更新用関数
void dispRobotState()
{
  M5.Lcd.setTextSize(1);
  // ゾーンの描画
  if ((robotState & STATE_ZONE) == 0)
  {
    M5.Lcd.fillRect(0, 0, 116, 20, TFT_RED);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setCursor(40, 16);
    M5.Lcd.print("RED");
  }
  else{
    M5.Lcd.fillRect(0, 0, 116, 20, TFT_BLUE);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setCursor(33, 16);
    M5.Lcd.print("BLUE");
  }

  // SDカード状態の描画
  if ((robotState & STATE_SD_INIT) == 0)
  {
    M5.Lcd.fillRect(119, 0, 50, 20, TFT_DARKGREY);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setCursor(129, 16);
    M5.Lcd.print("SD");
  }
  else{
    M5.Lcd.fillRect(119, 0, 50, 20, COLOR_NAVY);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setCursor(129, 16);
    M5.Lcd.print("SD");
  }

  // SDカード書き込み可のときの描画
  if ((robotState & STATE_SD_INIT) != 0)
  {
    M5.Lcd.fillRect(159, 0, 10, 20, TFT_YELLOW);
  }
  
  // LPMSの状態の描画
  if ((robotState & STATE_SD_INIT) == 0)
  {
    M5.Lcd.fillRect(172, 0, 70, 20, TFT_DARKGREY);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setCursor(182, 16);
    M5.Lcd.print("LPMS");
  }
  else{
    M5.Lcd.fillRect(172, 0, 70, 20, COLOR_NAVY);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setCursor(182, 16);
    M5.Lcd.print("LPMS");
  }
}
