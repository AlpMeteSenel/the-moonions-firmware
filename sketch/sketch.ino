#include <Arduino_RouterBridge.h>
#include <U8x8lib.h>
#include <Arduino_LSM6DSOX.h>

// Change to U8X8_SH1106_128X64_NONAME_HW_I2C if the module uses SH1106.
U8X8_SSD1306_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);
int lightState = -1;
LSM6DSOXClass movement(Wire1, 0x6A);
bool imuReady = false;

String readImu() {
    float ax, ay, az, gx, gy, gz;
    if (!imuReady || !movement.readAcceleration(ax, ay, az) ||
        !movement.readGyroscope(gx, gy, gz)) {
        return "";
    }
    String result;
    result.reserve(85);
    result += String(millis());
    const float values[] = {ax, ay, az, gx, gy, gz};
    for (float value : values) {
        result += ',';
        result += String(value, 5);
    }
    return result;
}


int readLight() {
    int total = 0;
    for (int sample = 0; sample < 16; ++sample) {
        total += analogRead(A0);
    }
    return total / 16;
}

void setLight(int state) {
    if (state == lightState) {
        return;
    }
    lightState = state;
    display.clearDisplay();
    display.drawString(0, 0, state < 0 ? "CALIBRA LUCE" : state ? "LUCE" : "BUIO");
}

void setup() {
    display.begin();
    display.setFont(u8x8_font_chroma48medium8_r);
    display.drawString(0, 0, "CALIBRA LUCE");
    imuReady = movement.begin();
    Bridge.begin();
    Bridge.provide("rover_read_imu", readImu);
    Bridge.provide("rover_read_light", readLight);
    Bridge.provide("rover_set_light", setLight);
}

void loop() {
    delay(20);
}
