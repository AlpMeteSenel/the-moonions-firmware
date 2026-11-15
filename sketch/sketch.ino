#include <Arduino_RouterBridge.h>
#include <U8x8lib.h>
#include <Arduino_LSM6DSOX.h>

// Change to U8X8_SH1106_128X64_NONAME_HW_I2C if the module uses SH1106.
U8X8_SSD1306_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);
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


void showLanderStatus(String status) {
    display.clearLine(3);
    display.drawString(0, 3, status.c_str());
}

void setup() {
    display.begin();
    display.setFont(u8x8_font_chroma48medium8_r);
    display.drawString(0, 0, "LANDER STATUS");
    showLanderStatus("STARTING");
    imuReady = movement.begin();
    Bridge.begin();
    Bridge.provide("rover_read_imu", readImu);
    Bridge.provide("rover_lander_status", showLanderStatus);
}

void loop() {
    delay(20);
}
