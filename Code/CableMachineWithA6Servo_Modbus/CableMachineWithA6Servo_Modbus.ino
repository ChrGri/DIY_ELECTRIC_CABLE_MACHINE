/*
 * Modbus Servo Control - Angepasst für ESP32-S3 Dev Module
 * Liest zyklisch Statusregister alle 10ms.
 * Verwendet korrigierte Modbus-Adressen (Parametername als Hex-Adresse).
 *
 * --- HINWEISE ZUR ESP32-S3-ANPASSUNG ---
 * 1. Strapping Pins: Sicher (GPIO 1, 4, 18, 21 verwendet).
 * 2. ADC-Pin: GPIO 1 (ADC1) für Potentiometer.
 *
 * Original-Kommentar: C03.40 = 0 digital reference
 */

#include <ModbusMaster.h>
#include <HardwareSerial.h> // Nötig für Serial2

// --- Pin-Definitionen (ANGEPASST FÜR S3) ---
const int potPin = 1; // GEÄNDERT: War 34. GPIO 1 ist ein sicherer ADC1-Pin.

#define RXD2_PIN 18      // RX Pin für Serial2 (an RXD des Moduls)
#define TXD2_PIN 21      // TX Pin für Serial2 (an TXD des Moduls)
#define SERVO_ON_PIN 4   // GPIO zur Steuerung von S-ON am Antrieb (CN1)

// --- Modbus Konfiguration ---
#define SERVO_DRIVE_SLAVE_ID 1 // Stationsnummer des Antriebs (C0A.00)

// --- Modbus Register Adressen (Hex) - KORRIGIERT (ParamName = Addr) ---
// Konfiguration
#define REG_CONTROL_MODE 0x0000     // C00.00 (16-bit)
#define REG_TORQUE_REF_SRC 0x0340   // C03.40 (16-bit) - KORRIGIERT
#define REG_TARGET_TORQUE 0x0341    // C03.41 (16-bit, Signed, Einheit 0.1%) - KORRIGIERT

// Status/Monitoring Register (U40 Gruppe, ParamName = Addr)
#define REG_SPEED_FEEDBACK 0x4001      // U40.01 (16-bit Signed, rpm) - KORRIGIERT
#define REG_TORQUE_FEEDBACK 0x4003     // U40.03 (16-bit Signed, 0.1%) - KORRIGIERT
#define REG_BUS_VOLTAGE 0x4006         // U40.06 (16-bit Unsigned, 0.1V) - KORRIGIERT
#define REG_RMS_CURRENT 0x400C         // U40.0C (16-bit Signed, 0.1A) - KORRIGIERT
#define REG_POSITION_FEEDBACK_L 0x4016 // U40.16 (Low Word, 32-bit Signed total, ref unit) - KORRIGIERT
#define REG_POSITION_FEEDBACK_H 0x4017 // U40.16 (High Word) - KORRIGIERT


// --- Globale Variablen ---
ModbusMaster node;
HardwareSerial ModbusSerial(2); // Nutze UART2

int16_t currentTargetTorque = 0;
bool servoIsEnabled = false;
bool modbusOk = false; // Status der Modbus-Verbindung

// Variablen zum Speichern der gelesenen Werte
int16_t actualSpeed = 0;
int16_t actualTorque = 0;
uint16_t busVoltage = 0;
int16_t rmsCurrent = 0;
int32_t actualPosition = 0;

// Zeitsteuerung für Modbus-Lesen und -Prüfen
unsigned long lastModbusReadTime = 0;
unsigned long lastModbusCheckTime = 0;
unsigned long lastPrintTime = 0;
const long modbusReadInterval = 10;     // Intervall für Status-Lesen in ms
const long modbusCheckInterval = 2000;  // Intervall für Verbindungs-Check in ms
const long printInterval = 250;         // Intervall für Serial Print in ms


// --- Funktion zum Überprüfen der Modbus-Verbindung ---
// Versucht das Control Mode Register (0x0000) zu lesen.
bool checkModbusConnection() {
    uint8_t result;
    result = node.readHoldingRegisters(REG_CONTROL_MODE, 1);

    if (result == node.ku8MBSuccess) {
        if (!modbusOk) {
            Serial.println("Modbus connection successful (Read 0x0000 OK).");
        }
        modbusOk = true;
        return true;
    } else {
        if (modbusOk || millis() < 5000) {
            Serial.print("Modbus connection check failed reading 0x0000! Error Code: 0x");
            Serial.println(result, HEX);
        }
        modbusOk = false;
        return false;
    }
}

// --- Funktion zum Lesen der Servo Statusdaten ---
bool readServoData() {
    uint8_t result;
    bool readSuccess = true;

    // 1. Lese Speed Feedback (Addr 0x4001)
    result = node.readHoldingRegisters(REG_SPEED_FEEDBACK, 1);
    if (result == node.ku8MBSuccess) {
        actualSpeed = node.getResponseBuffer(0);
    } else {
        Serial.print("Failed Speed! Code: 0x"); Serial.println(result, HEX);
        readSuccess = false;
    }
    delay(1);

    // 2. Lese Torque Feedback (Addr 0x4003)
    result = node.readHoldingRegisters(REG_TORQUE_FEEDBACK, 1);
    if (result == node.ku8MBSuccess) {
        actualTorque = node.getResponseBuffer(0);
    } else {
        Serial.print("Failed Torque! Code: 0x"); Serial.println(result, HEX);
        readSuccess = false;
    }
    delay(1);

    // 3. Lese Bus Voltage (Addr 0x4006)
    result = node.readHoldingRegisters(REG_BUS_VOLTAGE, 1);
    if (result == node.ku8MBSuccess) {
        busVoltage = node.getResponseBuffer(0);
    } else {
        Serial.print("Failed Voltage! Code: 0x"); Serial.println(result, HEX);
        readSuccess = false;
    }
    delay(1);

    // 4. Lese RMS Current (Addr 0x400C)
    result = node.readHoldingRegisters(REG_RMS_CURRENT, 1);
    if (result == node.ku8MBSuccess) {
        rmsCurrent = node.getResponseBuffer(0);
    } else {
        Serial.print("Failed Current! Code: 0x"); Serial.println(result, HEX);
        readSuccess = false;
    }
    delay(1);

    // 5. Lese Position Feedback (2 Register, Start Addr 0x4016)
    result = node.readHoldingRegisters(REG_POSITION_FEEDBACK_L, 2);
    if (result == node.ku8MBSuccess) {
        uint16_t lowWord = node.getResponseBuffer(0); // Value from 0x4016
        uint16_t highWord = node.getResponseBuffer(1); // Value from 0x4017
        actualPosition = (int32_t)((uint32_t)highWord << 16 | lowWord);
    } else {
        Serial.print("Failed Position! Code: 0x"); Serial.println(result, HEX);
        readSuccess = false;
    }

    if (!readSuccess) {
        modbusOk = false;
    }

    return readSuccess;
}


// --- Funktion zum Schreiben eines 16-bit Registers ---
bool writeRegister(uint16_t reg, int16_t value) {
    if (!modbusOk && millis() > 5000) {
        return false;
    }

    uint8_t result;
    node.setTransmitBuffer(0, value);
    result = node.writeSingleRegister(reg, 0);

    if (result == node.ku8MBSuccess) {
        return true;
    } else {
        Serial.print("Modbus write error Reg 0x");
        Serial.print(reg, HEX);
        Serial.print(", Code: 0x");
        Serial.println(result, HEX);
        modbusOk = false;
        digitalWrite(SERVO_ON_PIN, HIGH);
        servoIsEnabled = false;
        return false;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);
    Serial.println("\nModbus Servo Control Initializing (ESP32-S3)...");

    pinMode(potPin, INPUT);
    Serial.print("ADC Setup Complete. Potentiometer connected to GPIO ");
    Serial.println(potPin);

    pinMode(SERVO_ON_PIN, OUTPUT);
    digitalWrite(SERVO_ON_PIN, HIGH);
    servoIsEnabled = false;

    ModbusSerial.begin(115200, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
    if (!ModbusSerial) {
        Serial.println("!!! Failed to start Modbus Serial Port !!!");
        while (1) delay(100);
    } else {
        Serial.println("Modbus Serial Port OK.");
    }

    node.begin(SERVO_DRIVE_SLAVE_ID, ModbusSerial);

    Serial.println("Modbus Initialized. Checking connection...");
    delay(500);

    checkModbusConnection();
    if (!modbusOk) {
       Serial.println("WARNING: Initial Modbus check failed!");
    } else {
       Serial.println("Initial Modbus check successful.");
    }

    Serial.println("Configuring Drive (if connection allows)...");

    if (modbusOk) {
        delay(50);
        // Schreibe Control Mode (Addr 0x0000)
        if (!writeRegister(REG_CONTROL_MODE, 2)) { // 2 = Torque Mode
            Serial.println("Failed to set Control Mode!");
        }
        delay(50);
        // Schreibe Torque Ref Src (Addr 0x0340)
        if (!writeRegister(REG_TORQUE_REF_SRC, 0)) { // 0 = Internal Digital Setting
            Serial.println("Failed to set Torque Reference Source!");
        }
        delay(50);
         // Schreibe Target Torque (Addr 0x0341)
        if(!writeRegister(REG_TARGET_TORQUE, 0)) { // Start-Drehmoment auf 0 setzen
             Serial.println("Failed to set initial Torque to 0!");
        }
    } else {
         Serial.println("Skipping drive configuration due to initial connection failure.");
    }

     if (modbusOk) {
       Serial.println("Drive Configuration Attempted. Setup Complete. Ready.");
     } else {
       Serial.println("Drive Configuration might have failed. Setup incomplete.");
     }

    lastModbusReadTime = millis();
    lastModbusCheckTime = millis();
    lastPrintTime = millis();
}

void loop() {
    unsigned long currentTime = millis();

    // Periodische Überprüfung der Modbus-Verbindung
    if (currentTime - lastModbusCheckTime >= modbusCheckInterval) {
        lastModbusCheckTime = currentTime;
        if (!modbusOk) {
            checkModbusConnection();
        }
    }

    // Statusdaten alle 10ms lesen, nur wenn Verbindung OK ist
    if (modbusOk && (currentTime - lastModbusReadTime >= modbusReadInterval)) {
        lastModbusReadTime = currentTime;
        readServoData(); // Uses updated addresses
    }

    // Gelesene Daten zyklisch ausgeben
    if (currentTime - lastPrintTime >= printInterval) {
        lastPrintTime = currentTime;
        if (modbusOk) {
             Serial.printf("Pos:%ld | Spd:%d | Trq:%.1f%% | Cur:%.1fA | Vbus:%.1fV | TargTrq:%.1f%%\n",
                           actualPosition,
                           actualSpeed,
                           actualTorque * 0.1f,
                           rmsCurrent * 0.1f,
                           busVoltage * 0.1f,
                           currentTargetTorque * 0.1f);
        } else {
            Serial.println("Modbus not OK - Trying to reconnect...");
        }
    }


    // --- Drehmomentsteuerung basierend auf Potentiometer ---
    if (modbusOk) {
        int potValue = analogRead(potPin);
        int torqueReference_0p1_i32 = map(potValue, 0, 4095, 0, 500); // Max 50% Torque
        int16_t newTargetTorque = (int16_t)torqueReference_0p1_i32;

        if (!servoIsEnabled) {
            digitalWrite(SERVO_ON_PIN, LOW);
            servoIsEnabled = true;
            Serial.println("Servo Enabled.");
            delay(50);
        }

        // Schreibe Target Torque (Addr 0x0341)
        if (newTargetTorque != currentTargetTorque || !servoIsEnabled) {
             if (writeRegister(REG_TARGET_TORQUE, newTargetTorque)) {
                 currentTargetTorque = newTargetTorque;
             } else {
                 Serial.println("Failed to write torque. Servo Disabled due to write error.");
             }
        }

    } else {
        if (servoIsEnabled) {
            digitalWrite(SERVO_ON_PIN, HIGH);
            servoIsEnabled = false;
            Serial.println("Modbus connection lost. Servo Disabled.");
            currentTargetTorque = 0;
        }
        delay(100);
    }
     delay(1);
}

