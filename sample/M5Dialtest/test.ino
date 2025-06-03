/** Project CPP includes. */
#include "TLx493D_inc.hpp"
#include "M5Dial.h"
#include "USB.h"
#include "USBHIDMouse.h"

using namespace ifx::tlx493d;

/* Definition of the power pin and sensor objects for Kit2Go XMC1100 boards. */
const uint8_t POWER_PIN = 15; // XMC1100 : LED2

// some board swith multiple I2C need Wire --> Wire1
TLx493D_A1B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);

/** Definition of a counter variable. */
uint8_t count = 0;

USBHIDMouse Mouse;

// マウス制御用の定数
const float THRESHOLD = 5.0;      // 動作開始閾値
const float HYSTERESIS = 2.0;     // ヒステリシス幅
const float SCALE = 0.2;          // 磁気値からマウス移動量への変換係数

// 前回の状態を保存する変数
float lastX = 0;
float lastY = 0;
bool isMovingX = false;
bool isMovingY = false;
long encoderValue = 0;  // エンコーダーの値

// ヒステリシス制御付きの値変換関数
int convertToEncoderMove(float value, float lastValue, bool *isMoving) {
    // 絶対値が閾値を超えた場合に動作開始
    if (!*isMoving && abs(value) > THRESHOLD) {
        *isMoving = true;
    }
    // ヒステリシス考慮で動作停止
    else if (*isMoving && abs(value) < (THRESHOLD - HYSTERESIS)) {
        *isMoving = false;
    }

    if (*isMoving) {
        // 値を変化量に変換
        return (int)(value * SCALE);
    }
    return 0;
}

// ヒステリシス制御付きの値変換関数
int convertToMouseMove(float value, bool *isMoving) {
    if (!*isMoving && abs(value) > THRESHOLD) {
        *isMoving = true;
    } else if (*isMoving && abs(value) < (THRESHOLD - HYSTERESIS)) {
        *isMoving = false;
    }

    if (*isMoving) {
        float movement = value * SCALE;
        return (int)constrain(movement, -10, 10);
    }
    return 0;
}

void setup() {
    M5.begin();
    USB.begin();
    Mouse.begin();
    M5.Lcd.setTextSize(2);
    Serial.begin(115200);
    delay(3000);

    dut.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 0, 250000);
    dut.begin();

    // 初期画面設定
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE, BLACK); // 背景色を黒に設定
    
    // タイトルを表示
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.println("Magnetic Sensor");
    M5.Lcd.drawLine(10, 30, 230, 30, WHITE);
}

// ヘルパー関数を追加
void printBarGraph(const char* label, double value, double maxVal) {
    Serial.print(label);
    Serial.print(": [");
    
    // 値を0-20の範囲に正規化してバーの長さを決定
    int barLength = abs((int)(value * 20 / maxVal));
    barLength = min(barLength, 20); // 最大20文字
    
    for(int i = 0; i < 20; i++) {
        Serial.print(i < barLength ? "█" : " ");
    }
    Serial.print("] ");
    Serial.println(value);
}

// ヘルパー関数を追加
void displayValue(const char* label, double value, int y) {
    M5.Lcd.setCursor(10, y);
    M5.Lcd.printf("%s: %6.2f", label, value);
}

// LCD用バーグラフ描画関数
void drawBarGraph(const char* label, double value, double maxVal, int y) {
    const int barX = 70;       // バーの開始X座標を左寄りに
    const int barWidth = 130;  // バーの幅を調整
    const int barHeight = 16;  // バーの高さを少し小さく
    
    // ラベル領域をクリア
    M5.Lcd.fillRect(10, y, 50, barHeight, BLACK);
    // ラベルを表示
    M5.Lcd.setCursor(10, y + 2);
    M5.Lcd.printf("%s:", label);
    
    // バーの背景をクリア
    M5.Lcd.fillRect(barX, y, barWidth, barHeight, BLACK);
    // バーの枠を描画
    M5.Lcd.drawRect(barX, y, barWidth, barHeight, WHITE);
    
    // 値に応じたバーを描画
    int fillWidth = abs((int)(value * barWidth / maxVal));
    fillWidth = min(fillWidth, barWidth);
    
    uint16_t color = (value >= 0) ? BLUE : RED;
    if (fillWidth > 0) {
        M5.Lcd.fillRect(barX + 1, y + 1, fillWidth - 1, barHeight - 2, color);
    }
    
    // 数値表示領域をクリア
    M5.Lcd.fillRect(barX + barWidth + 5, y, 70, barHeight, BLACK);
    // 数値を表示
    M5.Lcd.setCursor(barX + barWidth + 5, y + 2);
    M5.Lcd.printf("%5.1f", value);
}

void loop() {
    double t, x, y, z;
    
    dut.getMagneticFieldAndTemperature(&x, &y, &z, &t);
    
    // マウス移動量を計算（ヒステリシス制御）
    int moveX = convertToMouseMove(x, &isMovingX);
    int moveY = convertToMouseMove(y, &isMovingY);
    
    // マウス移動（Y軸は反転）
    if (moveX != 0 || moveY != 0) {
        Mouse.move(moveX, -moveY);
    }
    
    lastX = x;
    lastY = y;
    
    // LCD画面にバーグラフ表示（Y座標を下げて配置）
    drawBarGraph("Temp", t, 100.0, 45);
    drawBarGraph("X", x, 100.0, 75);
    drawBarGraph("Y", y, 100.0, 105);
    drawBarGraph("Z", z, 100.0, 135);
    
    // エンコーダー値を表示
    M5.Lcd.setCursor(10, 165);
    M5.Lcd.printf("Enc: %d", (int)encoderValue);
    
    // シリアル出力は既存のまま
    Serial.println("\n--- Sensor Values ---");
    printBarGraph("Temp(C) ", t, 100.0);
    printBarGraph("X (mT)  ", x, 100.0);
    printBarGraph("Y (mT)  ", y, 100.0);
    printBarGraph("Z (mT)  ", z, 100.0);
    Serial.println();

    Serial.print("count : ");
    Serial.println(count);

    if( ++count == 4 ) {
        /** Reset does not work for W2BW : either drive strength too low or delay to stabilize critical. */
        dut.reset(true, dut.getSensorType() != TLx493D_A1B6_e);
        count = 0;
    }
    
    delay(10);  // マウス制御の更新を早くするため
}