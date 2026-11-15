#include <Arduino_RouterBridge.h>
#include <U8x8lib.h>
#include <Arduino_LSM6DSOX.h>

// Change to U8X8_SH1106_128X64_NONAME_HW_I2C if the module uses SH1106.
U8X8_SSD1306_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);
LSM6DSOXClass qwiicDefault(Wire1, 0x6A);
LSM6DSOXClass qwiicAlternate(Wire1, 0x6B);
LSM6DSOXClass headerDefault(Wire, 0x6A);
LSM6DSOXClass headerAlternate(Wire, 0x6B);
LSM6DSOXClass* movement = nullptr;
uint32_t lastProbe = 0;
String imuConnection = "Waiting for IMU probe";

int readChipId(TwoWire& bus, uint8_t address) {
    bus.beginTransmission(address);
    bus.write(0x0F);  // WHO_AM_I
    if (bus.endTransmission(false) != 0 || bus.requestFrom(address, uint8_t{1}) != 1) {
        return -1;
    }
    return bus.read();
}

void detectImu() {
    lastProbe = millis();
    struct Candidate {
        TwoWire& bus;
        uint8_t address;
        LSM6DSOXClass& driver;
        const char* name;
    };
    const Candidate candidates[] = {
        {Wire1, 0x6A, qwiicDefault, "Qwiic 0x6A"},
        {Wire1, 0x6B, qwiicAlternate, "Qwiic 0x6B"},
        {Wire, 0x6A, headerDefault, "Header I2C 0x6A"},
        {Wire, 0x6B, headerAlternate, "Header I2C 0x6B"},
    };
    imuConnection = "";
    for (const auto& candidate : candidates) {
        candidate.bus.begin();
        const int id = readChipId(candidate.bus, candidate.address);
        if ((id == 0x6C || id == 0x69) && candidate.driver.begin()) {
            movement = &candidate.driver;
            imuConnection = candidate.name;
            return;
        }
        if (imuConnection.length()) {
            imuConnection += "; ";
        }
        imuConnection += candidate.name;
        imuConnection += id < 0 ? " no response" : " chip ID 0x" + String(id, HEX);
    }
}

String readImuConnection() {
    return imuConnection;
}

String readImu() {
    if (movement == nullptr && millis() - lastProbe >= 1000) {
        detectImu();
    }
    if (movement == nullptr) {
        return "";
    }
    float ax, ay, az, gx, gy, gz;
    if (movement->readAcceleration(ax, ay, az) != 1 || movement->readGyroscope(gx, gy, gz) != 1) {
        movement = nullptr;
        imuConnection = "IMU read failed";
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
    Bridge.begin();
    Bridge.provide("rover_read_imu", readImu);
    Bridge.provide("rover_imu_connection", readImuConnection);
    Bridge.provide("rover_lander_status", showLanderStatus);
}

void loop() {
    delay(20);
}
