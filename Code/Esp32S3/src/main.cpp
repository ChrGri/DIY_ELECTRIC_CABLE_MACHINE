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
#define REG_TARGET_SPEED 0x0321        // C03.21 *** NEU HINZUGEFÜGT ***
#define REG_TORQUE_REF_SRC 0x0340      // C03.40
#define REG_TARGET_TORQUE 0x0341       // C03.41
#define REG_MODBUS_SERVO_ON 0x0411     // Servo Enable/Disable (Write)
#define REG_DI5_FUNCTION 0x0410        // C04.10
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
int16_t currentTargetTorque = 0;
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

// --- Homing State (NEU) ---
enum HomingState {
    HOMING_IDLE,
    HOMING_START,
    HOMING_WAIT_FOR_RUNNING, // <-- NEUER ZUSTAND
    HOMING_MOVING_SLOW,
    HOMING_DONE
};
volatile HomingState homingState = HOMING_IDLE;
int32_t homingPosition = 0;
const int16_t HOMING_SPEED_RPM = 60; // Homing-Geschwindigkeit 60 U/min
const int16_t HOMING_TORQUE_THRESHOLD = 100; // 10.0% Drehmoment (als "Strom"-Schwellenwert)

// NEUE Timer-Variablen für Homing
unsigned long homingStartTime = 0;
const long HOMING_START_TIMEOUT = 2000; // 2 Sekunden Warten auf "Running"


// Zeitsteuerung
unsigned long lastModbusReadTime = 0;
unsigned long lastModbusCheckTime = 0;
unsigned long lastWsSendTime = 0;
const long modbusReadInterval = 50; // **** ERHÖHT ****
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

// --- HTML für Hauptseite (mit Log-Fenster - *** GEÄNDERT ***) ---
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
    /* *** NEUER STYLE für E-Stop *** */
    .btn-estop { 
        background-color: #ff0000; /* Leuchtend Rot */
        color: white; 
        font-weight: bold; 
        width: 100%; /* Volle Breite */
        margin-top: 10px;
        padding: 15px; /* Größerer Knopf */
        font-size: 1.2em;
        margin-right: 0;
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
      <label for="torqueSlider">Zieldrehmoment (%):</label>
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
  var servoTargetState = false; // User's desired state
  var logTextArea = null;
  const MAX_LOG_LINES = 100; // Limit number of log lines displayed

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
    document.getElementById('estopBtn').addEventListener('click', onEstopClick); // *** NEU ***
    updateButtonStates(false, false); // Initial state assuming disabled and not homing
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
      // Add timestamp
      const now = new Date();
      const timeString = now.toTimeString().split(' ')[0]; // HH:MM:SS
      logTextArea.value += timeString + ': ' + message + '\n';
      // Limit log lines
      let lines = logTextArea.value.split('\n');
      if (lines.length > MAX_LOG_LINES) {
          logTextArea.value = lines.slice(lines.length - MAX_LOG_LINES).join('\n');
      }
      logTextArea.scrollTop = logTextArea.scrollHeight; // Auto-scroll
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

      // Handle Log Messages
      if (data.type === 'log') {
        logToConsole(data.message);
        return;
      }
      
      if (data.type === 'homingStatus') {
        logToConsole('Homing Status: ' + data.message);
        // Re-enable homing button on finish/fail
        if (data.status === 'finished' || data.status === 'failed') {
            document.getElementById('homeBtn').disabled = false;
            document.getElementById('homeBtn').classList.remove('btn-disabled');
        }
        return;
      }

      // Handle Status Updates
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
    logToConsole("Slider Change - Sending Torque: " + (targetTorque / 10.0).toFixed(1) + " %");
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

  // *** NEUE FUNKTION ***
  function onEstopClick(event) {
    logToConsole("!!! EMERGENCY STOP Clicked !!!");
    
    // Zustand auf Client-Seite sofort zurücksetzen
    servoTargetState = false;
    targetTorque = 0;
    document.getElementById('torqueSlider').value = 0;
    document.getElementById('torqueValue').textContent = '0.0 %';

    // E-Stop-Befehl an ESP senden
    websocket.send(JSON.stringify({command: 'eStop'}));
  }

  // *** GEÄNDERTE FUNKTION ***
  function updateButtonStates(isServoActuallyEnabled, homingInProgress) {
     let modbusIsOk = document.getElementById('modbusStatus').textContent === 'OK';

     // Enable/Disable buttons
     document.getElementById('enableBtn').disabled = isServoActuallyEnabled || homingInProgress;
     document.getElementById('enableBtn').classList.toggle('btn-disabled', isServoActuallyEnabled || homingInProgress);
     document.getElementById('disableBtn').disabled = !isServoActuallyEnabled || homingInProgress;
     document.getElementById('disableBtn').classList.toggle('btn-disabled', !isServoActuallyEnabled || homingInProgress);
     
     // Homing button
     document.getElementById('homeBtn').disabled = isServoActuallyEnabled || !modbusIsOk || homingInProgress;
     document.getElementById('homeBtn').classList.toggle('btn-disabled', isServoActuallyEnabled || !modbusIsOk || homingInProgress);

     // E-Stop button (Immer aktiv, außer Modbus ist offline)
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
    // node.setTransmitBuffer(0, value);
    result = node.writeSingleRegister(reg, value);
    if (result != node.ku8MBSuccess) {
        logToBrowser("MB Write FAIL: Reg=0x%04X, Val=%d, Code=0x%X", reg, value, result);
        modbusOk = false;
        actualServoStatus = 0; servoIsEnabledTarget = false; servoIsEnabledActual = false;
        modbusConsecutiveErrors = MAX_MODBUS_ERRORS; // Mark connection bad immediately on write fail
        return false;
    }
    modbusConsecutiveErrors = 0; // Reset error counter on successful write
    // delay(20); // Kleine Pause nach erfolgreichem Schreiben -> Entfernt, könnte Probleme machen
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
        // Status wird durch readServoData aktualisiert oder writeRegister Fehlerbehandlung
        delay(50); // Optional: Kurze Pause nach Fehler
        return false;
    }
}

// Deaktiviert Servo über Modbus
bool disableServoModbus() {
    logToBrowser("Attempting to disable Servo via Modbus (0x0411 = 0)...");
    bool success = writeRegister(REG_MODBUS_SERVO_ON, 0); // Try even if modbusOk=false
    if (!success && modbusOk) { logToBrowser("-> Modbus disable command FAILED."); }
    else if (success) { logToBrowser("-> Modbus disable command sent successfully."); }

    // Setze Zieldrehmoment auf 0, wenn deaktiviert wird (versuche es trotzdem)
    if(currentTargetTorque != 0) {
        if(writeRegister(REG_TARGET_TORQUE, 0)) { // Try setting torque to 0
            currentTargetTorque = 0;
            logToBrowser("-> Set target torque to 0 after disable.");
        } else if (modbusOk) { logToBrowser("MB: Failed to set torque to 0 after disable."); }
    }

    // Update internal status immediately if command failed or connection lost
    if (!success || !modbusOk) {
        actualServoStatus = (actualServoStatus == 3) ? 3 : 0; // Fault or Not Ready
        servoIsEnabledActual = false;
    }
    // delay(50); // Optional: Kurze Pause nach Disable
    return success;
}

// Prüft die Modbus Verbindung (wird seltener aufgerufen)
bool checkModbusConnection() {
    uint8_t result;
    result = node.readHoldingRegisters(REG_CONTROL_MODE, 1);
    if (result == node.ku8MBSuccess) {
        if (!modbusOk) logToBrowser("MB Connection Check OK (Read 0x0000 successful).");
        modbusOk = true;
        modbusConsecutiveErrors = 0; // Reset error counter
        return true;
    } else {
        // Log only if status changed or during startup
        if (modbusOk || millis() < 6000) {
            logToBrowser("MB Connection Check FAIL reading 0x0000! Code: 0x%X", result);
        }
        modbusOk = false;
        actualServoStatus = 0; servoIsEnabledTarget = false; servoIsEnabledActual = false;
        modbusConsecutiveErrors = MAX_MODBUS_ERRORS; // Assume max errors if check fails
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
    uint16_t tempStatus = actualServoStatus; // Store previous status to log changes

    // --- Read Sequence ---
    // Read Servo Status
    result = node.readHoldingRegisters(REG_SERVO_STATUS, 1);
    if (result == node.ku8MBSuccess) actualServoStatus = node.getResponseBuffer(0);
    else { /*logToBrowser("MB Rd Fail ServoStatus (0x410A): 0x%X", result);*/ readSuccessCurrentCycle = false; } // Temporarily disable flooding log

    // Read DI Status
    result = node.readHoldingRegisters(REG_DI_STATUS, 1);
     if (result == node.ku8MBSuccess) diStatus = node.getResponseBuffer(0);
     else { /*logToBrowser("MB Rd Fail DI Status (0x4004): 0x%X", result);*/ readSuccessCurrentCycle = false; }

    // Read Speed
    result = node.readHoldingRegisters(REG_SPEED_FEEDBACK, 1);
    if (result == node.ku8MBSuccess) actualSpeed = node.getResponseBuffer(0);
    else { /*logToBrowser("MB Rd Fail Spd (0x4001): 0x%X", result);*/ readSuccessCurrentCycle = false; }

    // Read Torque
    result = node.readHoldingRegisters(REG_TORQUE_FEEDBACK, 1);
    if (result == node.ku8MBSuccess) actualTorque = node.getResponseBuffer(0);
    else { /*logToBrowser("MB Rd Fail Trq (0x4003): 0x%X", result);*/ readSuccessCurrentCycle = false; }

    // Read Voltage
    result = node.readHoldingRegisters(REG_BUS_VOLTAGE, 1);
    if (result == node.ku8MBSuccess) busVoltage = node.getResponseBuffer(0);
    else { /*logToBrowser("MB Rd Fail VBus (0x4006): 0x%X", result);*/ readSuccessCurrentCycle = false; }

    // Read Current
    result = node.readHoldingRegisters(REG_RMS_CURRENT, 1);
    if (result == node.ku8MBSuccess) rmsCurrent = node.getResponseBuffer(0);
    else { /*logToBrowser("MB Rd Fail Cur (0x400C): 0x%X", result);*/ readSuccessCurrentCycle = false; }

    // Read Position (2 registers)
    result = node.readHoldingRegisters(REG_POSITION_FEEDBACK_L, 2);
    if (result == node.ku8MBSuccess) {
        actualPosition = (int32_t)((uint32_t)node.getResponseBuffer(1) << 16 | node.getResponseBuffer(0));
    } else {
        /*logToBrowser("MB Rd Fail Pos (0x4016): 0x%X", result);*/ readSuccessCurrentCycle = false;
    }
    // --- End Read Sequence ---

    // Update overall status based on errors
    if (!readSuccessCurrentCycle) {
        modbusConsecutiveErrors++;
        // Log only first few errors or when exceeding threshold
        if (modbusConsecutiveErrors <= 2 || modbusConsecutiveErrors == MAX_MODBUS_ERRORS) {
             logToBrowser("Modbus read cycle failed (%d consecutive)", modbusConsecutiveErrors);
        }

        if (modbusConsecutiveErrors >= MAX_MODBUS_ERRORS) {
            if (modbusOk) { // Log only when status changes to FAIL
                logToBrowser(">>> Too many consecutive Modbus read errors, setting status to FAIL <<<");
            }
            modbusOk = false;
            actualServoStatus = 0; servoIsEnabledTarget = false; servoIsEnabledActual = false;
        }
        return false; // Indicate failure in this cycle
    } else {
        // Success
        if (!modbusOk && modbusConsecutiveErrors == 0) { // Log only when status changes to OK
             logToBrowser(">>> Modbus communication OK <<<");
        }
        modbusConsecutiveErrors = 0; // Reset error counter
        modbusOk = true;
        // Update actual enabled state based on read status
        servoIsEnabledActual = (actualServoStatus == 2);
        // Log if servo status changed
        if(tempStatus != actualServoStatus) logToBrowser("Servo Status Changed (0x410A) = %d (0=NR,1=RD,2=RUN,3=FLT)", actualServoStatus);
        return true; // Indicate success in this cycle
    }
}


// --- WebSocket Event Handler (*** GEÄNDERT ***) ---
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
            wsJsonTx["homingInProgress"] = (homingState != HOMING_IDLE); // *** NEU ***
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
                            // *** GEÄNDERT: constrain(..., 0, 2000) ***
                            int16_t reqTorque = wsJsonRx["value"]; reqTorque = constrain(reqTorque, 0, 2000); 
                            if(currentTargetTorque != reqTorque) {
                                currentTargetTorque = reqTorque;
                                Serial.printf("WS: Received setTorque: %d (%.1f %%)\n", currentTargetTorque, currentTargetTorque/10.0);
                                if (servoIsEnabledActual && modbusOk) { writeRegister(REG_TARGET_TORQUE, currentTargetTorque); }
                            }
                        }
                    } else if (strcmp(command, "enableServo") == 0) {
                        Serial.println("WS: Received enableServo command.");
                        servoIsEnabledTarget = true;
                    } else if (strcmp(command, "disableServo") == 0) {
                        Serial.println("WS: Received disableServo command.");
                        servoIsEnabledTarget = false;
                        currentTargetTorque = 0; // Reset torque on disable command
                    } else if (strcmp(command, "getStatus") == 0) {
                        Serial.println("WS: Received getStatus command.");
                         wsJsonTx.clear(); wsJsonTx["type"] = "status"; wsJsonTx["modbusOk"] = modbusOk; wsJsonTx["servoEnabled"] = servoIsEnabledActual;
                         wsJsonTx["servoStatus"] = actualServoStatus; wsJsonTx["diStatus"] = diStatus; wsJsonTx["pos"] = actualPosition;
                         wsJsonTx["spd"] = actualSpeed; wsJsonTx["trq"] = actualTorque; wsJsonTx["cur"] = rmsCurrent; wsJsonTx["vbus"] = busVoltage;
                         wsJsonTx["homingInProgress"] = (homingState != HOMING_IDLE); // *** NEU ***
                         { String jsonString; serializeJson(wsJsonTx, jsonString); client->text(jsonString); }
                    } else if (strcmp(command, "setDI5Func") == 0) {
                         if (wsJsonRx.containsKey("value")) {
                            int16_t func = wsJsonRx["value"];
                            Serial.printf("WS: Received setDI5Func command: %d\n", func);
                            if (modbusOk) { writeRegister(REG_DI5_FUNCTION, func); }
                         }
                     // *** NEU: Homing-Befehl ***
                     } else if (strcmp(command, "startHoming") == 0) {
                         Serial.println("WS: Received startHoming command.");
                         if (modbusOk && !servoIsEnabledActual && homingState == HOMING_IDLE) {
                             homingState = HOMING_START;
                             logToBrowser("Homing sequence initiated...");
                         } else {
                             logToBrowser("Cannot start homing: Servo is enabled, Modbus is offline, or homing already in progress.");
                             // Send failure back to client to re-enable button
                             wsJsonTx.clear(); wsJsonTx["type"] = "homingStatus"; wsJsonTx["status"] = "failed"; wsJsonTx["message"] = "Homing rejected.";
                             String jsonString; serializeJson(wsJsonTx, jsonString); client->text(jsonString);
                         }
                     // *** NEUER E-Stop Befehl ***
                     } else if (strcmp(command, "eStop") == 0) {
                         Serial.println("WS: Received EMERGENCY STOP command!");
                         logToBrowser("!!! EMERGENCY STOP Received !!!");
                         
                         // Erzwinge alle Zustände auf AUS
                         servoIsEnabledTarget = false;
                         currentTargetTorque = 0; 
                         homingState = HOMING_IDLE; // Homing sofort abbrechen
                         
                         writeRegister(REG_CONTROL_MODE, 2); // Drehmomentmodus wiederherstellen
                         writeRegister(REG_TARGET_SPEED, 0); // Zielgeschwindigkeit löschen

                         // Sende sofort den Deaktivierungsbefehl
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
        logToBrowser("Configuring Drive...");
        disableServoModbus(); // Ensure servo starts disabled
        delay(100);
        if (!writeRegister(REG_CONTROL_MODE, 2)) logToBrowser("Failed to set Control Mode!");
        delay(50);
        if (!writeRegister(REG_TORQUE_REF_SRC, 0)) logToBrowser("Failed to set Torque Ref Source!");
        delay(50);
        if (!writeRegister(REG_TARGET_TORQUE, 0)) logToBrowser("Failed to set initial Torque to 0!");
        // Read DI5 Function setting
        uint8_t res = node.readHoldingRegisters(REG_DI5_FUNCTION, 1);
        if(res == node.ku8MBSuccess) {
            logToBrowser("Current DI5 Function (C04.10 / 0x0410) = %d (1=S-ON, 0=None)", node.getResponseBuffer(0));
        } else {
             logToBrowser("Failed to read DI5 function (0x0410). Code: 0x%X", res);
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
    homingState = HOMING_IDLE; // Sicherstellen, dass Homing im IDLE-Zustand startet
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

// --- Haupt-Loop der App (*** GEÄNDERT ***) ---
void appLoop() {
    unsigned long currentTime = millis();
    // static bool enableCmdSent = false; // (MOVED TO GLOBAL)

    // 1. Modbus Verbindung prüfen (wenn nicht ok und Intervall abgelaufen)
    if (!modbusOk && (currentTime - lastModbusCheckTime >= modbusCheckInterval)) {
        lastModbusCheckTime = currentTime;
        checkModbusConnection(); // Versucht Verbindung wiederherzustellen
        if (modbusOk) {
            // Nach erfolgreichem Reconnect sicherstellen, dass Servo aus ist
            disableServoModbus();
            enableCmdSent = false; // Reset command sent flag
        }
    }

    // 2. Modbus Daten lesen (häufig), nur wenn Verbindung OK (oder Fehlerzähler < Max)
    if (modbusOk || modbusConsecutiveErrors < MAX_MODBUS_ERRORS) {
        if (currentTime - lastModbusReadTime >= modbusReadInterval) {
            lastModbusReadTime = currentTime;
            readServoData(); // Liest Status und aktualisiert modbusOk / Fehlerzähler
        }
    }

    // *** NEU: Homing State Machine (hat Vorrang) ***
    if (homingState != HOMING_IDLE) {
        if (!modbusOk) {
            logToBrowser("Homing FAILED: Modbus connection lost.");
            homingState = HOMING_IDLE;
            // Benachrichtige UI
            wsJsonTx.clear(); wsJsonTx["type"] = "homingStatus"; wsJsonTx["status"] = "failed"; wsJsonTx["message"] = "Homing FAILED: Modbus lost.";
            { String jsonString; serializeJson(wsJsonTx, jsonString); ws.textAll(jsonString); }
        }

        switch (homingState) {
            case HOMING_START:
                logToBrowser("Homing: Setting Speed Mode (1) and Target Speed (%d rpm)...", HOMING_SPEED_RPM);
                if (writeRegister(REG_CONTROL_MODE, 1) &&         // Set Speed Mode
                    writeRegister(REG_TARGET_SPEED, HOMING_SPEED_RPM)) {
                    
                    logToBrowser("Homing: Enabling servo...");
                    if (enableServoModbus()) {
                        logToBrowser("Homing: Servo enable command sent. Waiting for 'Running' status...");
                        homingStartTime = millis(); // Timeout-Timer starten
                        homingState = HOMING_WAIT_FOR_RUNNING; // <-- Zum Wartezustand wechseln
                    } else {
                        logToBrowser("Homing FAILED: Could not enable servo.");
                        writeRegister(REG_CONTROL_MODE, 2); // Torque-Modus wiederherstellen
                        homingState = HOMING_IDLE; // Abort
                    }
                } else {
                    logToBrowser("Homing FAILED: Could not set speed mode/target.");
                    homingState = HOMING_IDLE; // Abort
                }
                break;

            // *** NEUER ZUSTAND ***
            case HOMING_WAIT_FOR_RUNNING:
                if (servoIsEnabledActual) { // Servo meldet Status 2 ("Running")
                    logToBrowser("Homing: Servo is 'Running'. Now monitoring for stall.");
                    homingState = HOMING_MOVING_SLOW; // Jetzt mit der Überwachung beginnen
                } else if (actualServoStatus == 3) { // Servo meldet Fehler
                    logToBrowser("Homing FAILED: Servo faulted while trying to start.");
                    homingState = HOMING_IDLE;
                } else if (millis() - homingStartTime > HOMING_START_TIMEOUT) { // Zeit abgelaufen
                    logToBrowser("Homing FAILED: Servo did not enter 'Running' state (Timeout).");
                    disableServoModbus(); // Versuchen, wieder zu deaktivieren
                    writeRegister(REG_CONTROL_MODE, 2); // Torque-Modus wiederherstellen
                    homingState = HOMING_IDLE;
                }
                // Sonst: Warten, bis eine der obigen Bedingungen eintritt
                break;

            case HOMING_MOVING_SLOW:
                // Dieser Block wird jetzt nur erreicht, wenn der Servo bestätigt hat, dass er läuft
                if (servoIsEnabledActual) { // Prüfen, ob der Servo noch läuft
                    if (abs(actualTorque) > HOMING_TORQUE_THRESHOLD) { 
                        logToBrowser("Homing: Stall detected (Torque > 10.0%%) at position %d. Stopping.", actualPosition);
                        homingPosition = actualPosition; // Position speichern
                        homingState = HOMING_DONE;
                    }
                } else {
                    // Servo hat unerwartet gestoppt (NACHDEM er lief)
                    if (actualServoStatus != 3) { // Wenn es kein Fehler war
                        logToBrowser("Homing FAILED: Servo stopped unexpectedly before stall.");
                    } else {
                        logToBrowser("Homing FAILED: Servo faulted during homing.");
                    }
                    homingState = HOMING_IDLE; // Abort
                }
                break;

            case HOMING_DONE:
                logToBrowser("Homing: Disabling servo and restoring Torque Mode (2)...");
                disableServoModbus(); // Motor stoppen
                writeRegister(REG_CONTROL_MODE, 2); // Drehmomentmodus wiederherstellen
                writeRegister(REG_TARGET_SPEED, 0); // Zielgeschwindigkeit löschen
                
                logToBrowser("Homing Finished. Position set to %d.", homingPosition);
                // HIER KÖNNTE man die Position als Nullpunkt setzen, falls gewünscht
                
                // Sende Abschlussnachricht an die Web-UI
                wsJsonTx.clear(); wsJsonTx["type"] = "homingStatus"; wsJsonTx["status"] = "finished";
                wsJsonTx["message"] = "Homing complete. Position: " + String(homingPosition);
                { String jsonString; serializeJson(wsJsonTx, jsonString); ws.textAll(jsonString); }

                homingState = HOMING_IDLE; // Zurück zum Leerlauf
                break;
            
            default:
                homingState = HOMING_IDLE;
                writeRegister(REG_CONTROL_MODE, 2); // Drehmomentmodus wiederherstellen
                writeRegister(REG_TARGET_SPEED, 0); // Zielgeschwindigkeit löschen
                break;
        }
    }


    // 3. Servo Enable/Disable Logik (Ziel vs. Aktuell - Send command only once per change)
    // *** Diese Logik nur ausführen, wenn KEIN Homing aktiv ist ***
    if (homingState == HOMING_IDLE) {
        if (modbusOk) {
            // Condition to send ENABLE command: Target is ON, Actual is OFF
            if (servoIsEnabledTarget && !servoIsEnabledActual) {
                // Only send if the servo is READY (status 1) AND command hasn't been sent yet
                if (actualServoStatus == 1 && !enableCmdSent) {
                     // *** ADDED LOGGING HERE ***
                     logToBrowser("Enable Condition Met: Target=ON, Actual=OFF, Status=1, CmdSent=FALSE -> Sending Enable Command...");
                    if(enableServoModbus()) { // Attempt to enable
                       enableCmdSent = true; // Mark as sent, wait for status update via readServoData
                    }
                    // If sending fails, enableCmdSent remains false, retry next suitable cycle
                }
                // Add logging to understand why enable might not be called
                // Log roughly every 2 seconds if stuck trying to enable
                else if (!enableCmdSent && (currentTime % 2000 < modbusReadInterval) ) { // Check approx every 2 sec
                     logToBrowser("Enable Check: Target=ON, Actual=OFF, Status=%d, CmdSent=%s -> Conditions not met.",
                                   actualServoStatus, enableCmdSent ? "true" : "false");
                }


            // Condition to send DISABLE command: Target is OFF, Actual is ON
            } else if (!servoIsEnabledTarget && servoIsEnabledActual) {
                 // logToBrowser("Enable Check: Target=OFF, Actual=ON -> Disabling..."); // Log disable condition - Can be noisy
                if (disableServoModbus()) { // Attempt to disable
                    enableCmdSent = false; // Reset sent flag after successful disable
                }

            // Condition where target and actual match (Reset/Set flags if needed)
            } else {
                 // If target is OFF and actual is OFF, ensure sent flag is reset
                 if (!servoIsEnabledTarget && !servoIsEnabledActual) {
                      if (enableCmdSent) { // Log only when resetting
                          // logToBrowser("State Match: Target=OFF, Actual=OFF -> Resetting enableCmdSent flag.");
                          enableCmdSent = false;
                      }
                 }
                 // If target is ON and actual is ON, ensure sent flag is set (correct state reached)
                 else if (servoIsEnabledTarget && servoIsEnabledActual) {
                     if (!enableCmdSent) { // Log only when setting
                          // logToBrowser("State Match: Target=ON, Actual=ON -> Setting enableCmdSent flag.");
                          enableCmdSent = true;
                     }
                 }
            }
        } // end if(modbusOk)
         else {
            // Modbus not OK -> Ensure internal state reflects disabled
            if (servoIsEnabledActual || servoIsEnabledTarget || enableCmdSent) { // Reset if any state indicates "ON" attempt
                 servoIsEnabledActual = false;
                 servoIsEnabledTarget = false;
                 actualServoStatus = 0; // Force status to Not Ready visually if comms lost
                 enableCmdSent = false; // Reset command sent flag if connection lost
                 // logToBrowser("Modbus Lost: Forcing internal state OFF and resetting enableCmdSent."); // Can be noisy
            }
        }
    } // end if(homingState == HOMING_IDLE)


    // 4. Daten an WebSocket Clients senden
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
            wsJsonTx["homingInProgress"] = (homingState != HOMING_IDLE); // *** NEU ***
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