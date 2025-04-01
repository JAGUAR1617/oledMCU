#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <U8g2lib.h>

// Hardware I2C for MPU6050 & BMP280
#define SDA_MPU_BMP D2
#define SCL_MPU_BMP D1

// Software I2C for OLED
#define SDA_OLED D6
#define SCL_OLED D5

// Button pin
#define BUTTON_PIN D3  

// Sensor objects
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

// Use Software I2C for OLED
U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, SCL_OLED, SDA_OLED, U8X8_PIN_NONE);

// Variables
double InputX, InputY, temperature, pressure, altitude;
#define SEALEVELPRESSURE_HPA (1013.25)

// OLED screen size
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Button state & modes
int displayMode = 0; // 0 = Sensor Data, 1 = Spirit Level, 2 = Eyes Mode

void setup() {
    Serial.begin(9600);
    Wire.begin(SDA_MPU_BMP, SCL_MPU_BMP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    Serial.println("🔍 Scanning I2C Devices...");
    scanI2CDevices();

    // Initialize MPU6050
    if (!mpu.begin(0x68)) {
        Serial.println("❌ MPU6050 NOT FOUND at 0x68! Trying 0x69...");
        if (!mpu.begin(0x69)) {
            Serial.println("❌ MPU6050 STILL NOT FOUND! Check wiring.");
            while (1);
        }
    }
    Serial.println("✅ MPU6050 Initialized!");

    // Initialize BMP280
    if (!bmp.begin(0x76)) {
        Serial.println("❌ BMP280 NOT FOUND at 0x76! Trying 0x77...");
        if (!bmp.begin(0x77)) {
            Serial.println("❌ BMP280 STILL NOT FOUND! Check wiring.");
            while (1);
        }
    }
    Serial.println("✅ BMP280 Initialized!");

    u8g2.begin();
    Serial.println("✅ OLED Initialized!");
}

void loop() {
    static unsigned long lastSensorUpdate = 0;
    static bool lastButtonState = HIGH;

    unsigned long now = millis();

    // Check for button press (Debounce)
    bool buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && lastButtonState == HIGH) {  
        displayMode = (displayMode + 1) % 3;  // Cycle through 3 modes
        Serial.println(displayMode == 0 ? "📊 Sensor Data Mode" :
                       displayMode == 1 ? "🟢 Spirit Level Mode" :
                       "👁️ Eye Mode");
        delay(50);
    }
    lastButtonState = buttonState;

    // Update sensors every 2 seconds
    if (now - lastSensorUpdate >= 500) {  
        lastSensorUpdate = now;
        readSensors();
    }

    // Refresh display
    if (displayMode == 0) {
        drawSensorData();
    } else if (displayMode == 1) {
        drawMovingCircle();
    } else {
        drawEyes();
    }
}

void readSensors() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    InputX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    InputY = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;

    temperature = bmp.readTemperature();
    pressure = bmp.readPressure() / 100.0F;
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);


    Serial.print("temp = "); Serial.println(temperature);
    Serial.print("pressure = "); Serial.println(pressure);
    Serial.print("altitute = "); Serial.println(altitude);
    Serial.print("InputX = "); Serial.println(InputX);
    Serial.print("InputY = "); Serial.println(InputY);
    Serial.println("----------------------------");
}

// 🎯 Spirit Level Mode (Moving Circle)
void drawMovingCircle() {
    u8g2.clearBuffer();

    int centerX = SCREEN_WIDTH / 2;
    int centerY = SCREEN_HEIGHT / 2;

    int circleX = centerX + (InputY * (SCREEN_WIDTH / 4) / 90);
    int circleY = centerY + (InputX * (SCREEN_HEIGHT / 4) / 90);

    circleX = constrain(circleX, 5, SCREEN_WIDTH - 5);
    circleY = constrain(circleY, 5, SCREEN_HEIGHT - 5);

    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(centerX - 30, 10);
    u8g2.print("Spirit Level");

    u8g2.drawCircle(centerX, centerY, 20, U8G2_DRAW_ALL);
    u8g2.drawDisc(circleX, circleY, 5);
    
    u8g2.sendBuffer();
}

// 📊 Sensor Data Mode
void drawSensorData() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    u8g2.setCursor(10, 10);  
    u8g2.print("Temp: ");  
    u8g2.print(temperature);  
    u8g2.print(" C");

    u8g2.setCursor(10, 30);  
    u8g2.print("Pressure: ");  
    u8g2.print(pressure);  
    u8g2.print(" hPa");

    u8g2.setCursor(10, 50);  
    u8g2.print("Altitude: ");  
    u8g2.print(altitude);  
    u8g2.print(" m");

    u8g2.sendBuffer();
}

// 👁️ Eye Mode (Animated Eyes)
void drawEyes() {
    u8g2.clearBuffer();

    int baseX1 = SCREEN_WIDTH / 4;
    int baseX2 = SCREEN_WIDTH * 3 / 4;
    int baseY = SCREEN_HEIGHT / 2;
    int baseX = SCREEN_WIDTH / 2;

    int eyeWidth = 20;
    int eyeHeight = 8;

    int offsetX = map(InputY, -90, 90, -5, 5);
    int offsetY = map(InputX, -90, 90, -5, 5);

    int shakeEffect = abs(InputX) > 30 || abs(InputY) > 30 ? random(-2, 2) : 0;

    u8g2.drawCircle(baseX1 - eyeWidth / 4 + offsetX + shakeEffect, baseY - eyeHeight / 4 + offsetY + shakeEffect, 20);
    u8g2.drawCircle(baseX2 - eyeWidth / 4 + offsetX + shakeEffect, baseY - eyeHeight / 4 + offsetY + shakeEffect, 20);

    // Left Eye
    u8g2.drawBox(baseX1 - eyeWidth / 2 + offsetX + shakeEffect, baseY - eyeHeight / 2 + offsetY + shakeEffect, eyeWidth, eyeHeight);
    u8g2.drawBox(baseX1 - eyeWidth / 4 + offsetX + shakeEffect, baseY - eyeHeight / 4 + offsetY + shakeEffect, eyeWidth / 2, eyeHeight / 2);

    // Right Eye
    u8g2.drawBox(baseX2 - eyeWidth / 2 + offsetX + shakeEffect, baseY - eyeHeight / 2 + offsetY + shakeEffect, eyeWidth, eyeHeight);
    u8g2.drawBox(baseX2 - eyeWidth / 4 + offsetX + shakeEffect, baseY - eyeHeight / 4 + offsetY + shakeEffect, eyeWidth / 2, eyeHeight / 2);

    u8g2.drawCircle(baseX + offsetX / 2 + shakeEffect, baseY + 5 + offsetY / 2 + shakeEffect, 4);

    u8g2.sendBuffer();
}

// 🔍 Scan I2C devices
void scanI2CDevices() {
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("✅ I2C device found at 0x");
            Serial.println(address, HEX);
        }
    }
    Serial.println("✅ I2C Scan Complete.");
}
