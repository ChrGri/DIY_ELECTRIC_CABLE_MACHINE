/*
 * Modbus Servo Control via Web Interface - ESP32-S3
 *
 * Implements a simple WiFi Manager, Modbus control, WebSocket interface,
 * and sends log messages to the web browser via WebSocket.
 *
 * Modbus Addresses based on user feedback (ParamName = Hex Addr).
 * Servo Enable/Disable via Modbus Register 0x0411.
 * Monitors correct Servo Status Register (U41.0A -> 0x410A).
 * Increased Modbus read interval and improved error handling.
 * Added detailed logging for enableCmdSent flag.
 *
 * *** MODIFIED ***
 * - Increased torque slider max to 200% (value 2000)
 * - Added Homing functionality (Button, WS command, State Machine)
 * - Added Homing race condition fix (WAIT_FOR_RUNNING)
 * - Added EMERGENCY STOP button
 * - Added writeRegister32bit() helper function for 32-bit registers like C06.08
 * - Changed FFB logic to use drive's internal software limits (C06.07, C06.08)
 * - Added deactivation of Out of Control Protection (C06.20)
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// --- Pin-Definitionen (ESP32-S3) ---
#define RXD2_PIN 18 // Modbus Serial2 RX
#define TXD2_PIN 21 // Modbus Serial2 TX

// --- WiFi Configuration ---
Preferences preferences;
String savedSSID = "";
String savedPassword = "";
const char *apSSID = "ServoSetup";

// --- Modbus Konfiguration ---
#define SERVO_DRIVE_SLAVE_ID 1
ModbusMaster node;
HardwareSerial ModbusSerial(2);

// --- Modbus Register Adressen (Hex) ---
#define REG_CONTROL_MODE 0x0000        // C00.00
#define REG_TARGET_SPEED 0x0321        // C03.21
#define REG_TORQUE_REF_SRC 0x0340      // C03.40
#define REG_TARGET_TORQUE 0x0341       // C03.41
#define REG_MODBUS_SERVO_ON 0x0411     // Servo Enable/Disable (Write)
#define REG_DI5_FUNCTION 0x0410        // C04.10
#define REG_SOFT_LIMIT_ENABLE 0x0607   // C06.07 (1=Enable +/- Limits)
#define REG_SOFT_LIMIT_NEG 0x0608      // C06.08 (32-bit Negative Limit) - Alias für Klarheit
#define REG_C06_08 REG_SOFT_LIMIT_NEG  // Beibehaltung des alten Namens zur Kompatibilität
#define REG_OUT_OF_CONTROL_PROT 0x0620 // C06.20 (Out of Control Protection Mode) *** NEU ***
#define REG_SPEED_FEEDBACK 0x4001      // U40.01
#define REG_TORQUE_FEEDBACK 0x4003     // U40.03
#define REG_DI_STATUS 0x4004           // U40.04
#define REG_BUS_VOLTAGE 0x4006         // U40.06
#define REG_RMS_CURRENT 0x400C         // U40.0C
#define REG_POSITION_FEEDBACK_L 0x4016 // U40.16 (Low)
#define REG_POSITION_FEEDBACK_H 0x4017 // U40.16 (High)
#define REG_SERVO_STATUS 0x410A        // U41.0A

// --- Webserver & WebSocket ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
StaticJsonDocument<300> wsJsonTx;
StaticJsonDocument<128> wsJsonRx;
#define MAX_LOG_MSG_LENGTH 150

// --- Globale Zustandsvariablen ---
bool servoIsEnabledTarget = false;
bool servoIsEnabledActual = false;
bool modbusOk = false;
int16_t currentTargetTorque = 0; // Represents the MAX torque from slider (0-2000)
int16_t actualSpeed = 0;
int16_t actualTorque = 0;
uint16_t busVoltage = 0;
int16_t rmsCurrent = 0;
int32_t actualPosition = 0;
uint16_t actualServoStatus = 0;
uint16_t diStatus = 0;
int modbusConsecutiveErrors = 0; // Zähler für Modbus-Fehler
const int MAX_MODBUS_ERRORS = 5; // Anzahl Fehler, bevor Verbindung als schlecht gilt
bool enableCmdSent = false;      // Track if enable command was sent

// --- Homing State ---
enum HomingState {
    HOMING_IDLE,
    HOMING_START,
    HOMING_WAIT_FOR_RUNNING,
    HOMING_MOVING_SLOW,
    HOMING_DONE
};
volatile HomingState homingState = HOMING_IDLE;
int32_t homingPosition = 0; // Wird aus Preferences geladen oder durch Homing gesetzt
const int16_t HOMING_SPEED_RPM = 60; // Homing-Geschwindigkeit 60 U/min
const int16_t HOMING_TORQUE_THRESHOLD = 100; // 10.0% Drehmoment (als "Strom"-Schwellenwert)

// Timer-Variablen für Homing
unsigned long homingStartTime = 0;
const long HOMING_START_TIMEOUT = 2000; // 2 Sekunden Warten auf "Running"

// Zeitsteuerung
unsigned long lastModbusReadTime = 0;
unsigned long lastModbusCheckTime = 0;
unsigned long lastWsSendTime = 0;
const long modbusReadInterval = 50;
const long modbusCheckInterval = 2000;
const long wsSendInterval = 100;
unsigned long wifiReconnectTimer = 0;

// Flag, ob wir im AP-Modus sind
bool isInAPMode = false;

// --- Hilfsfunktion für Logging ---
void logToBrowser(const char* format, ...) {
    char msgBuffer[MAX_LOG_MSG_LENGTH];
    va_list args;
    va_start(args, format);
    vsnprintf(msgBuffer, sizeof(msgBuffer), format, args);
    va_end(args);
    Serial.println(msgBuffer); // Immer an Serial
    if (!isInAPMode && ws.count() > 0 && WiFi.status() == WL_CONNECTED) {
        bool likely_in_ws_callback = false; // Vereinfachte Annahme
        if (!likely_in_ws_callback) {
             wsJsonTx.clear(); wsJsonTx["type"] = "log"; wsJsonTx["message"] = msgBuffer;
             String jsonString; serializeJson(wsJsonTx, jsonString); ws.textAll(jsonString);
        }
    }
}

// --- HTML für Hauptseite (mit Log-Fenster - Unverändert von letzter Version) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 Servo Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; padding: 15px; background-color: #f4f4f4; }
    h2 { color: #333; text-align: center; }
    .container { max-width: 500px; margin: auto; background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
    .control-group { margin-bottom: 20px; }
    label { display: block; margin-bottom: 5px; font-weight: bold; }
    input[type=range] { width: 100%; }
    .value-display { font-size: 1.2em; color: #007bff; text-align: center; margin-top: 5px; }
    .btn { padding: 10px 15px; font-size: 1em; cursor: pointer; border: none; border-radius: 5px; margin-right: 10px; }
    .btn-enable { background-color: #28a745; color: white; }
    .btn-disable { background-color: #dc3545; color: white; }
    .btn-home { background-color: #007bff; color: white; }
    .btn-disabled { background-color: #6c757d; color: white; cursor: not-allowed;}
    .btn-estop { 
        background-color: #ff0000; color: white; font-weight: bold; width: 100%; 
        margin-top: 10px; padding: 15px; font-size: 1.2em; margin-right: 0;
    }
    .status { margin-top: 20px; padding: 15px; background-color: #e9ecef; border-radius: 5px; }
    .status p { margin: 5px 0; }
    .status strong { color: #555; }
    .status-badge { padding: 3px 8px; border-radius: 4px; color: white; font-weight: bold; display: inline-block; min-width: 60px; text-align: center;}
    .status-run { background-color: #28a745; }
    .status-ready { background-color: #ffc107; color: #333;}
    .status-nr { background-color: #6c757d; }
    .status-fault { background-color: #dc3545; }
    .status-off { background-color: #6c757d; }
    .status-modbus-ok { background-color: #17a2b8; }
    .status-modbus-fail { background-color: #ffc107; color: #333; }
    .di-indicator { display: inline-block; width: 15px; height: 15px; border-radius: 50%; margin-left: 5px; vertical-align: middle;}
    .di-on { background-color: limegreen; }
    .di-off { background-color: lightgrey; }
    #logOutput { width: 98%; height: 150px; background-color: #333; color: #fff; font-family: monospace; font-size: 0.8em; border: 1px solid #ccc; border-radius: 4px; margin-top: 15px; overflow-y: scroll; padding: 5px; }
  </style>
</head>
<body>
  <div class="container">
    <h2>A6-RS Servo Steuerung</h2>
    <div class="control-group">
      <label for="torqueSlider">Maximales Drehmoment (%):</label> <!-- Label geändert -->
      <input type="range" id="torqueSlider" min="0" max="2000" value="0" step="10">
      <div id="torqueValue" class="value-display">0.0 %</div>
    </div>
    <div class="control-group">
      <button id="enableBtn" class="btn btn-enable">Aktivieren</button>
      <button id="disableBtn" class="btn btn-disable">Deaktivieren</button>
      <button id="homeBtn" class="btn btn-home">Homing</button>
    </div>
    <div class="control-group">
        <button id="estopBtn" class="btn btn-estop">EMERGENCY STOP</button>
    </div>
    <div class="status">
      <h4>Status</h4>
      <p>Modbus: <span id="modbusStatus" class="status-badge status-modbus-fail">Checking...</span></p>
      <p>Servo: <span id="servoStatus" class="status-badge status-off">Unknown</span> (<span id="servoStatusCode">?</span>)</p>
      <p>Position: <strong id="actualPosition">0</strong></p>
      <p>Geschwindigkeit: <strong id="actualSpeed">0</strong> rpm</p>
      <p>Drehmoment Ist: <strong id="actualTorque">0.0</strong> %</p>
      <p>Strom: <strong id="rmsCurrent">0.0</strong> A</p>
      <p>Bus Spannung: <strong id="busVoltage">0.0</strong> V</p>
      <p>DIs (1-8):
         <span id="di1" class="di-indicator di-off"></span> <span id="di2" class="di-indicator di-off"></span>
         <span id="di3" class="di-indicator di-off"></span> <span id="di4" class="di-indicator di-off"></span>
         <span id="di5" class="di-indicator di-off"></span> <span id="di6" class="di-indicator di-off"></span>
         <span id="di7" class="di-indicator di-off"></span> <span id="di8" class="di-indicator di-off"></span>
         (<span id="diValueHex">0x00</span>)
      </p>
    </div>
     <textarea id="logOutput" readonly></textarea>
  </div>
<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;
  var targetTorque = 0;
  var servoTargetState = false; 
  var logTextArea = null;
  const MAX_LOG_LINES = 100;

  window.addEventListener('load', onLoad);

  function onLoad(event) {
    logTextArea = document.getElementById('logOutput');
    initWebSocket();
    initUI();
  }

  function initUI() {
    document.getElementById('torqueSlider').addEventListener('input', onSliderInput);
    document.getElementById('torqueSlider').addEventListener('change', onSliderChange);
    document.getElementById('enableBtn').addEventListener('click', onEnableClick);
    document.getElementById('disableBtn').addEventListener('click', onDisableClick);
    document.getElementById('homeBtn').addEventListener('click', onHomeClick);
    document.getElementById('estopBtn').addEventListener('click', onEstopClick); 
    updateButtonStates(false, false); 
  }

  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage;
  }

  function logToConsole(message) {
      if (!logTextArea) return;
      const now = new Date();
      const timeString = now.toTimeString().split(' ')[0]; 
      logTextArea.value += timeString + ': ' + message + '\n';
      let lines = logTextArea.value.split('\n');
      if (lines.length > MAX_LOG_LINES) {
          logTextArea.value = lines.slice(lines.length - MAX_LOG_LINES).join('\n');
      }
      logTextArea.scrollTop = logTextArea.scrollHeight; 
  }

  function onOpen(event) {
    console.log('Connection opened');
    logToConsole('WebSocket Connection Opened');
    document.getElementById('modbusStatus').textContent = 'ESP Connected';
    document.getElementById('modbusStatus').className = 'status-badge status-modbus-ok';
    websocket.send(JSON.stringify({command: "getStatus"}));
  }

  function onClose(event) {
    console.log('Connection closed');
    logToConsole('WebSocket Connection Closed');
    document.getElementById('modbusStatus').textContent = 'ESP Disconnected';
    document.getElementById('modbusStatus').className = 'status-badge status-modbus-fail';
    document.getElementById('servoStatus').textContent = 'Unknown';
    document.getElementById('servoStatus').className = 'status-badge status-modbus-fail';
    setTimeout(initWebSocket, 2000);
  }

  function onMessage(event) {
    try {
      var data = JSON.parse(event.data);

      if (data.type === 'log') {
        logToConsole(data.message);
        return;
      }
      
      if (data.type === 'homingStatus') {
        logToConsole('Homing Status: ' + data.message);
        if (data.status === 'finished' || data.status === 'failed') {
            document.getElementById('homeBtn').disabled = false;
            document.getElementById('homeBtn').classList.remove('btn-disabled');
        }
        return;
      }

      if (data.type === 'status') {
        document.getElementById('actualPosition').textContent = data.pos;
        document.getElementById('actualSpeed').textContent = data.spd;
        document.getElementById('actualTorque').textContent = (data.trq / 10.0).toFixed(1);
        document.getElementById('rmsCurrent').textContent = (data.cur / 10.0).toFixed(1);
        document.getElementById('busVoltage').textContent = (data.vbus / 10.0).toFixed(1);

        document.getElementById('modbusStatus').textContent = data.modbusOk ? 'OK' : 'FAIL';
        document.getElementById('modbusStatus').className = data.modbusOk ? 'status-badge status-modbus-ok' : 'status-badge status-modbus-fail';

        let statusText = 'Unknown'; let statusClass = 'status-badge status-nr';
        switch(data.servoStatus) {
            case 0: statusText = 'Not Ready'; statusClass = 'status-badge status-nr'; break;
            case 1: statusText = 'Ready'; statusClass = 'status-badge status-ready'; break;
            case 2: statusText = 'Running'; statusClass = 'status-badge status-run'; break;
            case 3: statusText = 'Fault'; statusClass = 'status-badge status-fault'; break;
            default: statusText = 'Invalid (' + data.servoStatus + ')'; statusClass = 'status-badge status-fault'; break;
        }
        document.getElementById('servoStatus').textContent = statusText;
        document.getElementById('servoStatus').className = statusClass;
        document.getElementById('servoStatusCode').textContent = data.servoStatus;

        let isActuallyEnabled = (data.servoStatus === 2);
        let homingInProgress = data.homingInProgress || false;
        updateButtonStates(isActuallyEnabled, homingInProgress);

        let diVal = data.diStatus;
        document.getElementById('diValueHex').textContent = '0x' + diVal.toString(16).padStart(2, '0');
        for (let i = 1; i <= 8; i++) {
            let indicator = document.getElementById('di' + i);
            indicator.className = ((diVal >> (i - 1)) & 1) ? 'di-indicator di-on' : 'di-indicator di-off';
        }
      }
    } catch (e) {
      console.error('Error parsing JSON:', e, 'Data:', event.data);
      logToConsole('Error processing WebSocket message: ' + event.data);
    }
  }

  function onSliderInput(event) {
    targetTorque = parseInt(event.target.value);
    document.getElementById('torqueValue').textContent = (targetTorque / 10.0).toFixed(1) + ' %';
  }

 function onSliderChange(event) {
    targetTorque = parseInt(event.target.value);
    document.getElementById('torqueValue').textContent = (targetTorque / 10.0).toFixed(1) + ' %';
    logToConsole("Slider Change - Setting MAX Torque: " + (targetTorque / 10.0).toFixed(1) + " %"); // Log geändert
    websocket.send(JSON.stringify({command: 'setTorque', value: targetTorque}));
 }

  function onEnableClick(event) {
    logToConsole("Enable Button Clicked - Requesting Servo Enable");
    servoTargetState = true;
    websocket.send(JSON.stringify({command: 'enableServo'}));
  }

  function onDisableClick(event) {
    logToConsole("Disable Button Clicked - Requesting Servo Disable");
    servoTargetState = false;
    websocket.send(JSON.stringify({command: 'disableServo'}));
    targetTorque = 0;
    document.getElementById('torqueSlider').value = 0;
    document.getElementById('torqueValue').textContent = '0.0 %';
    websocket.send(JSON.stringify({command: 'setTorque', value: 0}));
  }
  
  function onHomeClick(event) {
    logToConsole("Homing Button Clicked - Requesting Homing Start");
    document.getElementById('homeBtn').disabled = true;
    document.getElementById('homeBtn').classList.add('btn-disabled');
    websocket.send(JSON.stringify({command: 'startHoming'}));
  }

  function onEstopClick(event) {
    logToConsole("!!! EMERGENCY STOP Clicked !!!");
    servoTargetState = false;
    targetTorque = 0;
    document.getElementById('torqueSlider').value = 0;
    document.getElementById('torqueValue').textContent = '0.0 %';
    websocket.send(JSON.stringify({command: 'eStop'}));
  }

  function updateButtonStates(isServoActuallyEnabled, homingInProgress) {
     let modbusIsOk = document.getElementById('modbusStatus').textContent === 'OK';

     document.getElementById('enableBtn').disabled = isServoActuallyEnabled || homingInProgress;
     document.getElementById('enableBtn').classList.toggle('btn-disabled', isServoActuallyEnabled || homingInProgress);
     document.getElementById('disableBtn').disabled = !isServoActuallyEnabled || homingInProgress;
     document.getElementById('disableBtn').classList.toggle('btn-disabled', !isServoActuallyEnabled || homingInProgress);
     
     document.getElementById('homeBtn').disabled = isServoActuallyEnabled || !modbusIsOk || homingInProgress;
     document.getElementById('homeBtn').classList.toggle('btn-disabled', isServoActuallyEnabled || !modbusIsOk || homingInProgress);

     document.getElementById('estopBtn').disabled = !modbusIsOk;
     document.getElementById('estopBtn').classList.toggle('btn-disabled', !modbusIsOk);
  }
</script>
</body>
</html>
)rawliteral";

// --- HTML für AP-Modus Konfigurationsseite (unverändert) ---
const char ap_mode_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><title>Servo WiFi Setup</title><meta name="viewport" content="width=device-width, initial-scale=1"><style>body{font-family:Arial,sans-serif;padding:15px;background-color:#f4f4f4;text-align:center;}h2{color:#333;}.container{max-width:400px;margin:30px auto;background:#fff;padding:20px;border-radius:8px;box-shadow:0 0 10px rgba(0,0,0,.1);}.form-group{margin-bottom:15px;text-align:left;}label{display:block;margin-bottom:5px;font-weight:bold;}input[type=text],input[type=password]{width:95%;padding:10px;border:1px solid #ccc;border-radius:4px;}.btn{padding:10px 20px;font-size:1em;cursor:pointer;border:none;border-radius:5px;background-color:#007bff;color:white;}.msg{margin-top:15px;color:green;font-weight:bold;}</style></head><body><div class="container"><h2>Servo WiFi Konfiguration</h2><p>Bitte geben Sie Ihre WLAN-Zugangsdaten ein.</p><form action="/save" method="POST"><div class="form-group"><label for="ssid">WLAN Name (SSID):</label><input type="text" id="ssid" name="ssid" required></div><div class="form-group"><label for="pass">WLAN Passwort:</label><input type="password" id="pass" name="pass" required></div><button type="submit" class="btn">Speichern & Neu starten</button></form><div id="message" class="msg"></div></div></body></html>
)rawliteral";


// --- Modbus Funktionen ---

// Schreibt ein 16-bit Register
bool writeRegister(uint16_t reg, int16_t value) {
    if (!modbusOk && millis() > 5000) { return false; }
    uint8_t result;
    result = node.writeSingleRegister(reg, value);
    if (result != node.ku8MBSuccess) {
        logToBrowser("MB Write FAIL: Reg=0x%04X, Val=%d, Code=0x%X", reg, value, result);
        modbusOk = false;
        actualServoStatus = 0; servoIsEnabledTarget = false; servoIsEnabledActual = false;
        modbusConsecutiveErrors = MAX_MODBUS_ERRORS;
        return false;
    }
    modbusConsecutiveErrors = 0;
    return true;
}

/**
 * @brief Schreibt einen 32-bit Wert (int32_t) in zwei aufeinanderfolgende 16-bit Modbus-Register.
 */
bool writeRegister32bit(uint16_t reg, int32_t value) {
    if (!modbusOk && millis() > 5000) { return false; }

    uint16_t lowWord = (uint16_t)(value & 0xFFFF);
    uint16_t highWord = (uint16_t)(value >> 16);

    node.setTransmitBuffer(0, lowWord);
    node.setTransmitBuffer(1, highWord);

    uint8_t result;
    result = node.writeMultipleRegisters(reg, 2); // Schreibt 2 Register ab Adresse 'reg'

    if (result != node.ku8MBSuccess) {
        logToBrowser("MB Write32 FAIL: Reg=0x%04X, Val=%ld, Code=0x%X", reg, value, result);
        modbusOk = false;
        actualServoStatus = 0; servoIsEnabledTarget = false; servoIsEnabledActual = false;
        modbusConsecutiveErrors = MAX_MODBUS_ERRORS;
        return false;
    }

    modbusConsecutiveErrors = 0;
    return true;
}


// Aktiviert Servo über Modbus
bool enableServoModbus() {
    logToBrowser("Attempting to enable Servo via Modbus (0x0411 = 1)...");
    if (writeRegister(REG_MODBUS_SERVO_ON, 1)) {
        logToBrowser("-> Modbus enable command sent successfully.");
        delay(50); // Optional: Kurze Pause nach Enable
        return true;
    } else {
        logToBrowser("-> Modbus enable command FAILED.");
        delay(50); // Optional: Kurze Pause nach Fehler
        return false;
    }
}

// Deaktiviert Servo über Modbus
bool disableServoModbus() {
    logToBrowser("Attempting to disable Servo via Modbus (0x0411 = 0)...");
    bool success = writeRegister(REG_MODBUS_SERVO_ON, 0); 
    if (!success && modbusOk) { logToBrowser("-> Modbus disable command FAILED."); }
    else if (success) { logToBrowser("-> Modbus disable command sent successfully."); }

    if(currentTargetTorque != 0) {
        // Versuche, das Drehmoment auf 0 zu setzen, ignoriere Fehler hier, da wir sowieso deaktivieren
        writeRegister(REG_TARGET_TORQUE, 0); 
        currentTargetTorque = 0; // Setze internen Wert trotzdem zurück
        logToBrowser("-> Set MAX torque to 0 after disable.");
    }

    if (!success || !modbusOk) {
        actualServoStatus = (actualServoStatus == 3) ? 3 : 0; 
        servoIsEnabledActual = false;
    }
    return success;
}

// Prüft die Modbus Verbindung (wird seltener aufgerufen)
bool checkModbusConnection() {
    uint8_t result;
    result = node.readHoldingRegisters(REG_CONTROL_MODE, 1);
    if (result == node.ku8MBSuccess) {
        if (!modbusOk) logToBrowser("MB Connection Check OK (Read 0x0000 successful).");
        modbusOk = true;
        modbusConsecutiveErrors = 0;
        return true;
    } else {
        if (modbusOk || millis() < 6000) {
            logToBrowser("MB Connection Check FAIL reading 0x0000! Code: 0x%X", result);
        }
        modbusOk = false;
        actualServoStatus = 0; servoIsEnabledTarget = false; servoIsEnabledActual = false;
        modbusConsecutiveErrors = MAX_MODBUS_ERRORS;
        return false;
    }
}

// Liest die Servo Statusdaten via Modbus
bool readServoData() {
    if (!modbusOk && modbusConsecutiveErrors >= MAX_MODBUS_ERRORS) {
        return false;
    }

    uint8_t result;
    bool readSuccessCurrentCycle = true;
    uint16_t tempStatus = actualServoStatus; 

    // --- Read Sequence ---
    result = node.readHoldingRegisters(REG_SERVO_STATUS, 1);
    if (result == node.ku8MBSuccess) actualServoStatus = node.getResponseBuffer(0);
    else { readSuccessCurrentCycle = false; } 

    result = node.readHoldingRegisters(REG_DI_STATUS, 1);
     if (result == node.ku8MBSuccess) diStatus = node.getResponseBuffer(0);
     else { readSuccessCurrentCycle = false; }

    result = node.readHoldingRegisters(REG_SPEED_FEEDBACK, 1);
    if (result == node.ku8MBSuccess) actualSpeed = node.getResponseBuffer(0);
    else { readSuccessCurrentCycle = false; }

    result = node.readHoldingRegisters(REG_TORQUE_FEEDBACK, 1);
    if (result == node.ku8MBSuccess) actualTorque = node.getResponseBuffer(0);
    else { readSuccessCurrentCycle = false; }

    result = node.readHoldingRegisters(REG_BUS_VOLTAGE, 1);
    if (result == node.ku8MBSuccess) busVoltage = node.getResponseBuffer(0);
    else { readSuccessCurrentCycle = false; }

    result = node.readHoldingRegisters(REG_RMS_CURRENT, 1);
    if (result == node.ku8MBSuccess) rmsCurrent = node.getResponseBuffer(0);
    else { readSuccessCurrentCycle = false; }

    result = node.readHoldingRegisters(REG_POSITION_FEEDBACK_L, 2);
    if (result == node.ku8MBSuccess) {
        actualPosition = (int32_t)((uint32_t)node.getResponseBuffer(1) << 16 | node.getResponseBuffer(0));
    } else {
        readSuccessCurrentCycle = false;
    }
    // --- End Read Sequence ---

    if (!readSuccessCurrentCycle) {
        modbusConsecutiveErrors++;
        if (modbusConsecutiveErrors <= 2 || modbusConsecutiveErrors == MAX_MODBUS_ERRORS) {
             logToBrowser("Modbus read cycle failed (%d consecutive)", modbusConsecutiveErrors);
        }
        if (modbusConsecutiveErrors >= MAX_MODBUS_ERRORS) {
            if (modbusOk) { 
                logToBrowser(">>> Too many consecutive Modbus read errors, setting status to FAIL <<<");
            }
            modbusOk = false;
            actualServoStatus = 0; servoIsEnabledTarget = false; servoIsEnabledActual = false;
        }
        return false; 
    } else {
        if (!modbusOk && modbusConsecutiveErrors == 0) { 
             logToBrowser(">>> Modbus communication OK <<<");
        }
        modbusConsecutiveErrors = 0; 
        modbusOk = true;
        servoIsEnabledActual = (actualServoStatus == 2);
        if(tempStatus != actualServoStatus) logToBrowser("Servo Status Changed (0x410A) = %d (0=NR,1=RD,2=RUN,3=FLT)", actualServoStatus);
        return true; 
    }
}


// --- WebSocket Event Handler ---
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WS Client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            wsJsonTx.clear(); wsJsonTx["type"] = "log"; wsJsonTx["message"] = "Client connected";
            { String jsonString; serializeJson(wsJsonTx, jsonString); client->text(jsonString); }
            // Send initial status
            wsJsonTx.clear(); wsJsonTx["type"] = "status"; wsJsonTx["modbusOk"] = modbusOk; wsJsonTx["servoEnabled"] = servoIsEnabledActual;
            wsJsonTx["servoStatus"] = actualServoStatus; wsJsonTx["diStatus"] = diStatus; wsJsonTx["pos"] = actualPosition;
            wsJsonTx["spd"] = actualSpeed; wsJsonTx["trq"] = actualTorque; wsJsonTx["cur"] = rmsCurrent; wsJsonTx["vbus"] = busVoltage;
            wsJsonTx["homingInProgress"] = (homingState != HOMING_IDLE); 
            { String jsonString; serializeJson(wsJsonTx, jsonString); client->text(jsonString); }
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WS Client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA: {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                data[len] = 0;
                wsJsonRx.clear();
                DeserializationError error = deserializeJson(wsJsonRx, (char*)data);
                if (error) { Serial.printf("deserializeJson() failed: %s\n", error.c_str()); return; }

                const char* command = wsJsonRx["command"];
                if (command) {
                    if (strcmp(command, "setTorque") == 0) {
                        if (wsJsonRx.containsKey("value")) {
                            int16_t reqTorque = wsJsonRx["value"]; 
                            reqTorque = constrain(reqTorque, 0, 2000); 
                            
                            if(currentTargetTorque != reqTorque) {
                                currentTargetTorque = reqTorque;
                                Serial.printf("WS: Set MAX Torque: %d (%.1f %%)\n", currentTargetTorque, currentTargetTorque/10.0);
                            }
                        }
                    } else if (strcmp(command, "enableServo") == 0) {
                        Serial.println("WS: Received enableServo command.");
                        servoIsEnabledTarget = true;
                    } else if (strcmp(command, "disableServo") == 0) {
                        Serial.println("WS: Received disableServo command.");
                        servoIsEnabledTarget = false;
                        currentTargetTorque = 0; // Reset max torque on disable command
                    } else if (strcmp(command, "getStatus") == 0) {
                        Serial.println("WS: Received getStatus command.");
                         wsJsonTx.clear(); wsJsonTx["type"] = "status"; wsJsonTx["modbusOk"] = modbusOk; wsJsonTx["servoEnabled"] = servoIsEnabledActual;
                         wsJsonTx["servoStatus"] = actualServoStatus; wsJsonTx["diStatus"] = diStatus; wsJsonTx["pos"] = actualPosition;
                         wsJsonTx["spd"] = actualSpeed; wsJsonTx["trq"] = actualTorque; wsJsonTx["cur"] = rmsCurrent; wsJsonTx["vbus"] = busVoltage;
                         wsJsonTx["homingInProgress"] = (homingState != HOMING_IDLE); 
                         { String jsonString; serializeJson(wsJsonTx, jsonString); client->text(jsonString); }
                    } else if (strcmp(command, "setDI5Func") == 0) {
                         if (wsJsonRx.containsKey("value")) {
                            int16_t func = wsJsonRx["value"];
                            Serial.printf("WS: Received setDI5Func command: %d\n", func);
                            if (modbusOk) { writeRegister(REG_DI5_FUNCTION, func); }
                         }
                     } else if (strcmp(command, "startHoming") == 0) {
                         Serial.println("WS: Received startHoming command.");
                         if (modbusOk && !servoIsEnabledActual && homingState == HOMING_IDLE) {
                             homingState = HOMING_START;
                             logToBrowser("Homing sequence initiated...");
                         } else {
                             logToBrowser("Cannot start homing: Servo is enabled, Modbus is offline, or homing already in progress.");
                             wsJsonTx.clear(); wsJsonTx["type"] = "homingStatus"; wsJsonTx["status"] = "failed"; wsJsonTx["message"] = "Homing rejected.";
                             String jsonString; serializeJson(wsJsonTx, jsonString); client->text(jsonString);
                         }
                     } else if (strcmp(command, "eStop") == 0) {
                         Serial.println("WS: Received EMERGENCY STOP command!");
                         logToBrowser("!!! EMERGENCY STOP Received !!!");
                         
                         servoIsEnabledTarget = false;
                         currentTargetTorque = 0; 
                         homingState = HOMING_IDLE; 
                         
                         disableServoModbus(); 
                     }
                }
            }
        } break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

// --- Setup für den AP-Modus (unverändert) ---
void setupAPMode() {
    isInAPMode = true;
    Serial.println("\nStarting Access Point Mode...");
    WiFi.softAP(apSSID);
    IPAddress apIP = WiFi.softAPIP();
    Serial.print("AP IP address: "); Serial.println(apIP);
    Serial.print("Connect to WiFi '"); Serial.print(apSSID); Serial.println("' and navigate to http://192.168.4.1");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", ap_mode_html); });
    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
        String newSSID, newPassword; bool success = false;
        if (request->hasParam("ssid", true) && request->hasParam("pass", true)) {
            newSSID = request->getParam("ssid", true)->value(); newPassword = request->getParam("pass", true)->value();
            if (newSSID.length() > 0 && newPassword.length() > 0) {
                preferences.begin("wifi-creds", false); preferences.putString("ssid", newSSID); preferences.putString("password", newPassword); preferences.end();
                success = true; Serial.println("WiFi credentials saved.");
            } else { Serial.println("Received empty SSID or Password."); }
        } else { Serial.println("Missing SSID or Password parameter in request."); }
        String resp = "<html><head><title>WiFi Setup</title><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{font-family: Arial; text-align: center; margin-top: 50px;} .msg{font-weight: bold;} .ok{color: green;} .err{color: red;}</style></head><body><h2>WiFi Setup Status</h2>";
        if (success) { resp += "<p class='msg ok'>Credentials saved successfully!</p><p>ESP32 will restart in 5 seconds...</p>"; request->send(200, "text/html", resp); delay(5000); ESP.restart(); }
        else { resp += "<p class='msg err'>Failed to save credentials. Please try again.</p><p><a href='/'>Go Back</a></p>"; request->send(400, "text/html", resp); }
        resp += "</body></html>";
    });
     server.onNotFound([](AsyncWebServerRequest *request){ request->redirect("/"); });
    server.begin(); Serial.println("Configuration server started.");
}

// --- Setup für den normalen App-Betrieb (STA-Modus) ---
void setupApp() {
    isInAPMode = false;
    logToBrowser("\nStarting Application Setup (STA Mode)...");

    // Gespeicherte Homing-Position laden
    preferences.begin("servo", true); // read-only
    homingPosition = preferences.getLong("homingPos", 0); // Standard 0, falls nicht gesetzt
    preferences.end();
    logToBrowser("Loaded homing position: %d", homingPosition);

    // Modbus Setup
    ModbusSerial.begin(115200, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
    if (!ModbusSerial) { logToBrowser("!!! Failed to start Modbus Serial Port in STA Mode !!!"); delay(5000); ESP.restart(); }
    else { logToBrowser("Modbus Serial Port OK."); }
    node.begin(SERVO_DRIVE_SLAVE_ID, ModbusSerial);

    logToBrowser("Checking initial Modbus connection...");
    delay(500);
    checkModbusConnection();
    if (!modbusOk) logToBrowser("WARNING: Initial Modbus check failed!");
    else {
        logToBrowser("Configuring Drive for Torque Mode with Software Limits...");
        disableServoModbus(); // Ensure servo starts disabled
        delay(100);
        
        // Grundkonfiguration für Torque Mode
        if (!writeRegister(REG_CONTROL_MODE, 2)) logToBrowser("Failed to set Control Mode (2)!");
        delay(50);
        if (!writeRegister(REG_TORQUE_REF_SRC, 0)) logToBrowser("Failed to set Torque Ref Source (0)!");
        delay(50);
        if (!writeRegister(REG_TARGET_TORQUE, 0)) logToBrowser("Failed to set initial Torque to 0!");
        delay(50);

        // Software Limits setzen
        logToBrowser("Setting Negative Software Limit (C06.08) to %d...", homingPosition);
        if (!writeRegister32bit(REG_SOFT_LIMIT_NEG, homingPosition)) {
             logToBrowser("FAILED to write Negative Software Limit!");
        }
        delay(50); 
        logToBrowser("Enabling Software Limits (C06.07 = 1)...");
        if (!writeRegister(REG_SOFT_LIMIT_ENABLE, 1)) { // Wert 1 aktiviert +/- Limits
            logToBrowser("FAILED to enable Software Limits!");
        } else {
             logToBrowser("Software Limits enabled (Positive=0, Negative=%d)", homingPosition);
        }
        delay(50); // *** NEU ***

        // *** NEU: Out of Control Protection deaktivieren ***
        logToBrowser("Disabling Out of Control Protection (C06.20 = 0)...");
        if (!writeRegister(REG_OUT_OF_CONTROL_PROT, 0)) {
            logToBrowser("FAILED to disable Out of Control Protection!");
        } else {
             logToBrowser("Out of Control Protection disabled.");
        }
        
    }

    // Webserver & WebSocket Setup
    ws.onEvent(onWsEvent); server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", index_html); });
    server.onNotFound([](AsyncWebServerRequest *request){ request->send(404, "text/plain", "Not found"); });
    server.begin();
    logToBrowser("HTTP server started. Open browser to http://%s", WiFi.localIP().toString().c_str());

    // Initialisiere Timer und Zustände
    lastModbusReadTime = millis(); lastModbusCheckTime = millis(); lastWsSendTime = millis();
    servoIsEnabledTarget = false; servoIsEnabledActual = false; currentTargetTorque = 0; actualServoStatus = 0; modbusConsecutiveErrors = 0;
    homingState = HOMING_IDLE; 
}

// --- Haupt-Setup ---
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);
    Serial.println("\nBooting Servo Controller...");

    preferences.begin("wifi-creds", true); // read-only
    savedSSID = preferences.getString("ssid", ""); savedPassword = preferences.getString("password", "");
    preferences.end();

    bool connected = false;
    if (savedSSID.length() > 0 && savedPassword.length() > 0) {
        Serial.print("Trying saved credentials for: "); Serial.println(savedSSID);
        WiFi.mode(WIFI_STA); WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
        int t = 0; while (WiFi.status() != WL_CONNECTED && t < 20) { delay(500); Serial.print("."); t++; }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi Connected!"); Serial.print("IP: "); Serial.println(WiFi.localIP());
            connected = true; setupApp();
        } else { Serial.println("\nConnection failed!"); WiFi.disconnect(true); WiFi.mode(WIFI_OFF); }
    } else { Serial.println("No saved WiFi credentials."); }

    if (!connected) { setupAPMode(); }
}

// --- Haupt-Loop der App ---
void appLoop() {
    unsigned long currentTime = millis();

    // 1. Modbus Verbindung prüfen (wenn nicht ok und Intervall abgelaufen)
    if (!modbusOk && (currentTime - lastModbusCheckTime >= modbusCheckInterval)) {
        lastModbusCheckTime = currentTime;
        checkModbusConnection(); 
        if (modbusOk) {
            logToBrowser("Reconnected to Modbus. Re-applying settings...");
            disableServoModbus();
            enableCmdSent = false;
             delay(50); writeRegister(REG_CONTROL_MODE, 2);
             delay(50); writeRegister(REG_TORQUE_REF_SRC, 0);
             delay(50); writeRegister32bit(REG_SOFT_LIMIT_NEG, homingPosition);
             delay(50); writeRegister(REG_SOFT_LIMIT_ENABLE, 1);
             delay(50); writeRegister(REG_OUT_OF_CONTROL_PROT, 0); // Auch hier erneut setzen
        }
    }

    // 2. Modbus Daten lesen (häufig), nur wenn Verbindung OK (oder Fehlerzähler < Max)
    if (modbusOk || modbusConsecutiveErrors < MAX_MODBUS_ERRORS) {
        if (currentTime - lastModbusReadTime >= modbusReadInterval) {
            lastModbusReadTime = currentTime;
            readServoData(); 
        }
    }

    // 3. Homing State Machine (hat Vorrang)
    if (homingState != HOMING_IDLE) {
        if (!modbusOk) {
            logToBrowser("Homing FAILED: Modbus connection lost.");
            homingState = HOMING_IDLE;
            wsJsonTx.clear(); wsJsonTx["type"] = "homingStatus"; wsJsonTx["status"] = "failed"; wsJsonTx["message"] = "Homing FAILED: Modbus lost.";
            { String jsonString; serializeJson(wsJsonTx, jsonString); ws.textAll(jsonString); }
        }

        switch (homingState) {
            case HOMING_START:
                logToBrowser("Homing: Disabling Software Limits (C06.07 = 0)...");
                if (writeRegister(REG_SOFT_LIMIT_ENABLE, 0)) {
                    delay(50);
                    logToBrowser("Homing: Setting Speed Mode (1) and Target Speed (%d rpm)...", HOMING_SPEED_RPM);
                    if (writeRegister(REG_CONTROL_MODE, 1) && writeRegister(REG_TARGET_SPEED, HOMING_SPEED_RPM)) {
                        logToBrowser("Homing: Enabling servo...");
                        if (enableServoModbus()) {
                            logToBrowser("Homing: Servo enable command sent. Waiting for 'Running' status...");
                            homingStartTime = millis(); 
                            homingState = HOMING_WAIT_FOR_RUNNING; 
                        } else {
                            logToBrowser("Homing FAILED: Could not enable servo.");
                            writeRegister(REG_CONTROL_MODE, 2); 
                            writeRegister(REG_SOFT_LIMIT_ENABLE, 1); 
                            homingState = HOMING_IDLE; 
                        }
                    } else {
                        logToBrowser("Homing FAILED: Could not set speed mode/target.");
                        writeRegister(REG_SOFT_LIMIT_ENABLE, 1); 
                        homingState = HOMING_IDLE; 
                    }
                } else {
                    logToBrowser("Homing FAILED: Could not disable software limits.");
                    homingState = HOMING_IDLE;
                }
                break;

            case HOMING_WAIT_FOR_RUNNING:
                if (servoIsEnabledActual) { 
                    logToBrowser("Homing: Servo is 'Running'. Now monitoring for stall.");
                    homingState = HOMING_MOVING_SLOW; 
                } else if (actualServoStatus == 3) { 
                    logToBrowser("Homing FAILED: Servo faulted while trying to start.");
                    writeRegister(REG_SOFT_LIMIT_ENABLE, 1); 
                    homingState = HOMING_IDLE;
                } else if (millis() - homingStartTime > HOMING_START_TIMEOUT) { 
                    logToBrowser("Homing FAILED: Servo did not enter 'Running' state (Timeout).");
                    disableServoModbus(); 
                    writeRegister(REG_CONTROL_MODE, 2); 
                    writeRegister(REG_SOFT_LIMIT_ENABLE, 1); 
                    homingState = HOMING_IDLE;
                }
                break;

            case HOMING_MOVING_SLOW:
                if (servoIsEnabledActual) { 
                    if (abs(actualTorque) > HOMING_TORQUE_THRESHOLD) { 
                        logToBrowser("Homing: Stall detected (Torque > 10.0%%) at position %d. Stopping.", actualPosition);
                        homingPosition = actualPosition; 
                        homingState = HOMING_DONE;
                    }
                } else {
                    if (actualServoStatus != 3) { 
                        logToBrowser("Homing FAILED: Servo stopped unexpectedly before stall.");
                    } else {
                        logToBrowser("Homing FAILED: Servo faulted during homing.");
                    }
                    writeRegister(REG_SOFT_LIMIT_ENABLE, 1); 
                    homingState = HOMING_IDLE; 
                }
                break;

            case HOMING_DONE:
                logToBrowser("Homing: Disabling servo, restoring Torque Mode (2), and setting new software limit...");
                disableServoModbus(); 
                delay(50); 
                writeRegister(REG_CONTROL_MODE, 2); 
                delay(50);
                writeRegister(REG_TARGET_SPEED, 0); 
                delay(50);

                logToBrowser("Setting Negative Software Limit (C06.08) to new position %d...", homingPosition);
                if (writeRegister32bit(REG_SOFT_LIMIT_NEG, homingPosition)) {
                     logToBrowser("New Negative Software Limit set.");
                } else {
                     logToBrowser("FAILED to write new Negative Software Limit!");
                }
                delay(50);
                logToBrowser("Re-enabling Software Limits (C06.07 = 1)...");
                 if (writeRegister(REG_SOFT_LIMIT_ENABLE, 1)) {
                     logToBrowser("Software Limits re-enabled.");
                 } else {
                      logToBrowser("FAILED to re-enable Software Limits!");
                 }
                
                logToBrowser("Homing Finished. Position set to %d.", homingPosition);
                
                preferences.begin("servo", false); 
                preferences.putLong("homingPos", homingPosition);
                preferences.end();
                logToBrowser("Homing position %d saved to flash.", homingPosition);
                
                wsJsonTx.clear(); wsJsonTx["type"] = "homingStatus"; wsJsonTx["status"] = "finished";
                wsJsonTx["message"] = "Homing complete. Position: " + String(homingPosition);
                { String jsonString; serializeJson(wsJsonTx, jsonString); ws.textAll(jsonString); }

                homingState = HOMING_IDLE; 
                break;
            
            default:
                homingState = HOMING_IDLE;
                break;
        }
    }


    // 4. Servo Enable/Disable & Torque Senden (nur wenn kein Homing aktiv ist)
    if (homingState == HOMING_IDLE) {
        if (modbusOk) {
            
            // --- 4a. Enable/Disable Befehlslogik ---
            if (servoIsEnabledTarget && !servoIsEnabledActual) {
                if (actualServoStatus == 1 && !enableCmdSent) {
                     logToBrowser("Enable Condition Met: Target=ON, Actual=OFF, Status=1, CmdSent=FALSE -> Sending Enable Command...");
                    if(enableServoModbus()) { 
                       enableCmdSent = true; 
                    }
                }
                else if (!enableCmdSent && (currentTime % 2000 < modbusReadInterval) ) { 
                     logToBrowser("Enable Check: Target=ON, Actual=OFF, Status=%d, CmdSent=%s -> Conditions not met.",
                                   actualServoStatus, enableCmdSent ? "true" : "false");
                }
            } else if (!servoIsEnabledTarget && servoIsEnabledActual) {
                if (disableServoModbus()) { 
                    enableCmdSent = false; 
                }
            } else {
                 if (!servoIsEnabledTarget && !servoIsEnabledActual) {
                      if (enableCmdSent) { enableCmdSent = false; }
                 }
                 else if (servoIsEnabledTarget && servoIsEnabledActual) {
                     if (!enableCmdSent) { enableCmdSent = true; }
                 }
            }

            // --- 4b. Drehmoment senden (wenn aktiv) ---
            // Sendet jetzt immer den Wert vom Slider (currentTargetTorque).
            // Der Servo regelt selbst gegen die Software Limits.
            if (servoIsEnabledActual) {
                // Sende das am Slider eingestellte (maximale) Drehmoment
                writeRegister(REG_TARGET_TORQUE, currentTargetTorque);
                // WICHTIG: Die Software Limits im Servo verhindern, dass sich der Motor
                // über homingPosition hinausbewegt. Der Widerstand wird vom Servo selbst erzeugt.
            }
            // (Wenn servoIsEnabledActual == false ist, hat disableServoModbus() das Drehmoment bereits auf 0 gesetzt)

        } // end if(modbusOk)
         else {
            // Modbus not OK -> Ensure internal state reflects disabled
            if (servoIsEnabledActual || servoIsEnabledTarget || enableCmdSent) { 
                 servoIsEnabledActual = false;
                 servoIsEnabledTarget = false;
                 actualServoStatus = 0; 
                 enableCmdSent = false; 
            }
        }
    } // end if(homingState == HOMING_IDLE)


    // 5. Daten an WebSocket Clients senden
    if (currentTime - lastWsSendTime >= wsSendInterval) {
        lastWsSendTime = currentTime;
        if (ws.count() > 0) {
            wsJsonTx.clear();
            wsJsonTx["type"] = "status";
            wsJsonTx["modbusOk"] = modbusOk;
            wsJsonTx["servoEnabled"] = servoIsEnabledActual;
            wsJsonTx["servoStatus"] = actualServoStatus;
            wsJsonTx["diStatus"] = diStatus;
            wsJsonTx["pos"] = actualPosition;
            wsJsonTx["spd"] = actualSpeed;
            wsJsonTx["trq"] = actualTorque;
            wsJsonTx["cur"] = rmsCurrent;
            wsJsonTx["vbus"] = busVoltage;
            wsJsonTx["homingInProgress"] = (homingState != HOMING_IDLE); 
            { String jsonString; serializeJson(wsJsonTx, jsonString); ws.textAll(jsonString); }
        }
    }

    ws.cleanupClients();
    delay(50); // General delay added by user for stability
}

// --- Haupt-Loop ---
void loop() {
    if (isInAPMode) {
        delay(10); // AP Mode does very little in loop
    } else if (WiFi.status() == WL_CONNECTED) {
        appLoop(); // Run main application logic
        wifiReconnectTimer = 0; // Reset reconnect timer if connected
    } else {
        // STA Mode but connection lost
        if(wifiReconnectTimer == 0) {
            logToBrowser("WiFi connection lost. Attempting to reconnect...");
            wifiReconnectTimer = millis();
        }
        if (millis() - wifiReconnectTimer > 10000) {
             Serial.print(".");
             WiFi.disconnect(); // Explicitly disconnect before reconnect
             WiFi.reconnect();
             wifiReconnectTimer = millis();
        }
        // Ensure Modbus/Servo is off during disconnect
        if (modbusOk || servoIsEnabledActual) {
            disableServoModbus(); // Try to send disable command
            modbusOk = false;
            actualServoStatus = 0;
            servoIsEnabledActual = false;
            servoIsEnabledTarget = false;
            enableCmdSent = false; // Reset flag on disconnect
            homingState = HOMING_IDLE; // Homing bei WLAN-Verlust abbrechen
            logToBrowser("WiFi lost, Modbus communication stopped.");
        }
        delay(500); // Wait between checks
    }
}

