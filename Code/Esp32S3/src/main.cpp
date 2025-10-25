/*
 * Modbus Servo Control via Web Interface - ESP32-S3
 *
 * Connects to WiFi, hosts a web page with a slider for torque,
 * enable/disable buttons, and displays live servo data using WebSockets.
 * Controls the A6-RS servo via Modbus RTU on Serial2.
 *
 * Modbus Addresses based on user feedback (ParamName = Hex Addr).
 * Servo Enable/Disable via Modbus Register 0x0411.
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h> // For WebSocket communication

#include <WifiCredentials.h> // For WebSocket communication

// --- Pin-Definitionen (ESP32-S3) ---
const int potPin = 1; // Potentiometer (optional, Wert wird jetzt vom Webinterface überschrieben)
#define RXD2_PIN 18   // Modbus Serial2 RX
#define TXD2_PIN 21   // Modbus Serial2 TX


// --- Modbus Konfiguration ---
#define SERVO_DRIVE_SLAVE_ID 1
ModbusMaster node;
HardwareSerial ModbusSerial(2);

// --- Modbus Register Adressen (Hex) ---
#define REG_CONTROL_MODE 0x0000        // C00.00
#define REG_TORQUE_REF_SRC 0x0340      // C03.40
#define REG_TARGET_TORQUE 0x0341       // C03.41
#define REG_MODBUS_SERVO_ON 0x0411     // Servo Enable/Disable
#define REG_SPEED_FEEDBACK 0x4001      // U40.01
#define REG_TORQUE_FEEDBACK 0x4003     // U40.03
#define REG_BUS_VOLTAGE 0x4006         // U40.06
#define REG_RMS_CURRENT 0x400C         // U40.0C
#define REG_POSITION_FEEDBACK_L 0x4016 // U40.16 (Low)
#define REG_POSITION_FEEDBACK_H 0x4017 // U40.16 (High)

// --- Webserver & WebSocket ---
AsyncWebServer server(80);        // Webserver auf Port 80
AsyncWebSocket ws("/ws");         // WebSocket auf Pfad /ws
StaticJsonDocument<256> wsJsonTx; // JSON-Dokument zum Senden
StaticJsonDocument<128> wsJsonRx; // JSON-Dokument zum Empfangen

// --- Globale Zustandsvariablen ---
bool servoIsEnabledTarget = false; // Gewünschter Zustand (vom Webinterface)
bool servoIsEnabledActual = false; // Tatsächlicher Zustand (nach Modbus-Kommunikation)
bool modbusOk = false;
int16_t currentTargetTorque = 0; // Ziel-Drehmoment (vom Webinterface)

// Gelesene Servo-Daten
int16_t actualSpeed = 0;
int16_t actualTorque = 0;
uint16_t busVoltage = 0;
int16_t rmsCurrent = 0;
int32_t actualPosition = 0;

// Zeitsteuerung
unsigned long lastModbusReadTime = 0;
unsigned long lastModbusCheckTime = 0;
unsigned long lastWsSendTime = 0;
const long modbusReadInterval = 10;    // ms
const long modbusCheckInterval = 2000; // ms
const long wsSendInterval = 100;       // ms (Wie oft Daten an Webinterface senden)

// --- HTML-Seite ---
// Einfache HTML-Seite mit Slider, Buttons und Datenanzeige
// Styling mit minimalem CSS für bessere Lesbarkeit
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
    .btn-disabled { background-color: #6c757d; color: white; cursor: not-allowed;}
    .status { margin-top: 20px; padding: 15px; background-color: #e9ecef; border-radius: 5px; }
    .status p { margin: 5px 0; }
    .status strong { color: #555; }
    .status-badge { padding: 3px 8px; border-radius: 4px; color: white; font-weight: bold; }
    .status-on { background-color: #28a745; }
    .status-off { background-color: #dc3545; }
    .status-modbus-ok { background-color: #17a2b8; }
    .status-modbus-fail { background-color: #ffc107; color: #333; }
  </style>
</head>
<body>
  <div class="container">
    <h2>A6-RS Servo Steuerung</h2>

    <div class="control-group">
      <label for="torqueSlider">Zieldrehmoment (%):</label>
      <input type="range" id="torqueSlider" min="0" max="1000" value="0" step="10">
      <div id="torqueValue" class="value-display">0.0 %</div>
    </div>

    <div class="control-group">
      <button id="enableBtn" class="btn btn-enable">Aktivieren</button>
      <button id="disableBtn" class="btn btn-disable">Deaktivieren</button>
    </div>

    <div class="status">
      <h4>Status</h4>
      <p>Modbus: <span id="modbusStatus" class="status-badge status-modbus-fail">Checking...</span></p>
      <p>Servo: <span id="servoStatus" class="status-badge status-off">Disabled</span></p>
      <p>Position: <strong id="actualPosition">0</strong></p>
      <p>Geschwindigkeit: <strong id="actualSpeed">0</strong> rpm</p>
      <p>Drehmoment Ist: <strong id="actualTorque">0.0</strong> %</p>
      <p>Strom: <strong id="rmsCurrent">0.0</strong> A</p>
      <p>Bus Spannung: <strong id="busVoltage">0.0</strong> V</p>
    </div>
  </div>

<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;
  var targetTorque = 0;
  var servoTargetState = false; // false = disabled, true = enabled

  window.addEventListener('load', onLoad);

  function onLoad(event) {
    initWebSocket();
    initUI();
  }

  function initUI() {
    document.getElementById('torqueSlider').addEventListener('input', onSliderInput);
    document.getElementById('torqueSlider').addEventListener('change', onSliderChange); // Send on release
    document.getElementById('enableBtn').addEventListener('click', onEnableClick);
    document.getElementById('disableBtn').addEventListener('click', onDisableClick);
    updateButtonStates();
  }

  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage;
  }

  function onOpen(event) {
    console.log('Connection opened');
    document.getElementById('modbusStatus').textContent = 'ESP Connected';
    document.getElementById('modbusStatus').className = 'status-badge status-modbus-ok'; // Assume ok initially
     // Request initial state from ESP32
    websocket.send(JSON.stringify({command: "getStatus"}));
  }

  function onClose(event) {
    console.log('Connection closed');
    document.getElementById('modbusStatus').textContent = 'ESP Disconnected';
    document.getElementById('modbusStatus').className = 'status-badge status-modbus-fail';
    document.getElementById('servoStatus').textContent = 'Unknown';
    document.getElementById('servoStatus').className = 'status-badge status-modbus-fail';
    setTimeout(initWebSocket, 2000); // Try to reconnect every 2 seconds
  }

 function onMessage(event) {
    // console.log('Received:', event.data);
    try {
      var data = JSON.parse(event.data);
      if (data.type === 'status') {
        document.getElementById('actualPosition').textContent = data.pos;
        document.getElementById('actualSpeed').textContent = data.spd;
        document.getElementById('actualTorque').textContent = (data.trq / 10.0).toFixed(1);
        document.getElementById('rmsCurrent').textContent = (data.cur / 10.0).toFixed(1);
        document.getElementById('busVoltage').textContent = (data.vbus / 10.0).toFixed(1);

        if (data.modbusOk) {
          document.getElementById('modbusStatus').textContent = 'OK';
          document.getElementById('modbusStatus').className = 'status-badge status-modbus-ok';
        } else {
          document.getElementById('modbusStatus').textContent = 'FAIL';
          document.getElementById('modbusStatus').className = 'status-badge status-modbus-fail';
        }

        if (data.servoEnabled) {
          document.getElementById('servoStatus').textContent = 'Enabled';
          document.getElementById('servoStatus').className = 'status-badge status-on';
          servoTargetState = true; // Update local state if reported back
        } else {
          document.getElementById('servoStatus').textContent = 'Disabled';
          document.getElementById('servoStatus').className = 'status-badge status-off';
           servoTargetState = false; // Update local state
        }
        updateButtonStates(); // Update button enable/disable state
      } else if(data.type === 'torqueSet') {
        // Optional: Bestätigung oder UI-Update, dass Drehmoment gesetzt wurde
        // console.log("Torque set to: " + data.value);
      }
    } catch (e) {
      console.error('Error parsing JSON:', e);
      console.error('Received data:', event.data);
    }
  }

  function onSliderInput(event) {
    targetTorque = parseInt(event.target.value);
    document.getElementById('torqueValue').textContent = (targetTorque / 10.0).toFixed(1) + ' %';
  }

 function onSliderChange(event) {
    targetTorque = parseInt(event.target.value);
    document.getElementById('torqueValue').textContent = (targetTorque / 10.0).toFixed(1) + ' %';
    console.log("Slider Change - Sending Torque: " + targetTorque);
    // Send torque value via WebSocket
    websocket.send(JSON.stringify({command: 'setTorque', value: targetTorque}));
 }

  function onEnableClick(event) {
    console.log("Enable Clicked");
    servoTargetState = true;
    updateButtonStates();
    websocket.send(JSON.stringify({command: 'enableServo'}));
  }

  function onDisableClick(event) {
    console.log("Disable Clicked");
    servoTargetState = false;
    updateButtonStates();
    websocket.send(JSON.stringify({command: 'disableServo'}));
    // Set slider and target torque to 0 when disabling
    targetTorque = 0;
    document.getElementById('torqueSlider').value = 0;
    document.getElementById('torqueValue').textContent = '0.0 %';
    websocket.send(JSON.stringify({command: 'setTorque', value: 0}));
  }

  function updateButtonStates() {
     if (servoTargetState) {
        document.getElementById('enableBtn').disabled = true;
        document.getElementById('enableBtn').classList.add('btn-disabled');
        document.getElementById('disableBtn').disabled = false;
        document.getElementById('disableBtn').classList.remove('btn-disabled');
     } else {
        document.getElementById('enableBtn').disabled = false;
        document.getElementById('enableBtn').classList.remove('btn-disabled');
        document.getElementById('disableBtn').disabled = true;
        document.getElementById('disableBtn').classList.add('btn-disabled');
     }
  }

</script>
</body>
</html>
)rawliteral";

// --- Modbus Funktionen ---

// Schreibt ein 16-bit Register
bool writeRegister(uint16_t reg, int16_t value) {
    if (!modbusOk && millis() > 5000) {
        return false;
    }
    uint8_t result;
    node.setTransmitBuffer(0, value);
    result = node.writeSingleRegister(reg, 0);
    if (result != node.ku8MBSuccess) {
        Serial.print("Modbus write ERROR Reg 0x"); Serial.print(reg, HEX); Serial.print(" Val:"); Serial.print(value); Serial.print(" Code: 0x"); Serial.println(result, HEX);
        modbusOk = false;
        servoIsEnabledTarget = false; // Bei Schreibfehler -> Ziel = AUS
        servoIsEnabledActual = false; // Bei Schreibfehler -> Faktisch AUS
        return false;
    }
    return true;
}

// Aktiviert Servo über Modbus
bool enableServoModbus() {
    // Serial.println("MB: Enabling Servo..."); // Debug
    if (writeRegister(REG_MODBUS_SERVO_ON, 1)) {
        // Serial.println("MB: Enable cmd OK."); // Debug
        servoIsEnabledActual = true;
        return true;
    } else {
        Serial.println("MB: Enable cmd FAILED.");
        servoIsEnabledActual = false; // Konnte nicht aktiviert werden
        return false;
    }
}

// Deaktiviert Servo über Modbus
bool disableServoModbus() {
    // Serial.println("MB: Disabling Servo..."); // Debug
    // Versuche auch bei !modbusOk zu senden, falls möglich
    bool success = writeRegister(REG_MODBUS_SERVO_ON, 0);
    servoIsEnabledActual = false; // Servo ist (oder sollte sein) jetzt aus
    if (!success) {
        // Serial.println("MB: Disable cmd FAILED (maybe connection lost?)."); // Debug
    } else {
         // Serial.println("MB: Disable cmd OK."); // Debug
    }
    // Setze Zieldrehmoment auf 0, wenn deaktiviert wird
    if(currentTargetTorque != 0) {
        if(writeRegister(REG_TARGET_TORQUE, 0)) {
            currentTargetTorque = 0;
        } else {
             Serial.println("MB: Failed to set torque to 0 after disable.");
        }
    }
    return success; // Gibt zurück, ob der Befehl erfolgreich war
}

// Prüft die Modbus Verbindung
bool checkModbusConnection() {
    uint8_t result;
    result = node.readHoldingRegisters(REG_CONTROL_MODE, 1); // Lese C00.00
    if (result == node.ku8MBSuccess) {
        if (!modbusOk) Serial.println("MB: Connection Check OK.");
        modbusOk = true;
        return true;
    } else {
        if (modbusOk || millis() < 5000) Serial.printf("MB: Connection Check FAILED reading 0x0000! Code: 0x%X\n", result);
        modbusOk = false;
        if(servoIsEnabledTarget) { // Wenn Servo AN sein sollte, aber Verbindung weg ist
             // disableServoModbus(); // Versucht automatisch zu deaktivieren
        }
        servoIsEnabledTarget = false; // Ziel auf AUS setzen bei Verbindungsfehler
        servoIsEnabledActual = false;
        return false;
    }
}

// Liest die Servo Statusdaten via Modbus
bool readServoData() {
    uint8_t result;
    bool readSuccess = true;

    // Lese Speed, Torque, Voltage, Current (jeweils 1 Register)
    result = node.readHoldingRegisters(REG_SPEED_FEEDBACK, 1);
    if (result == node.ku8MBSuccess) actualSpeed = node.getResponseBuffer(0); else { Serial.printf("MB Rd Fail Spd:0x%X\n", result); readSuccess = false; }
    delay(1);

    result = node.readHoldingRegisters(REG_TORQUE_FEEDBACK, 1);
    if (result == node.ku8MBSuccess) actualTorque = node.getResponseBuffer(0); else { Serial.printf("MB Rd Fail Trq:0x%X\n", result); readSuccess = false; }
    delay(1);

    result = node.readHoldingRegisters(REG_BUS_VOLTAGE, 1);
    if (result == node.ku8MBSuccess) busVoltage = node.getResponseBuffer(0); else { Serial.printf("MB Rd Fail VBus:0x%X\n", result); readSuccess = false; }
    delay(1);

    result = node.readHoldingRegisters(REG_RMS_CURRENT, 1);
    if (result == node.ku8MBSuccess) rmsCurrent = node.getResponseBuffer(0); else { Serial.printf("MB Rd Fail Cur:0x%X\n", result); readSuccess = false; }
    delay(1);

    // Lese Position (2 Register)
    result = node.readHoldingRegisters(REG_POSITION_FEEDBACK_L, 2);
    if (result == node.ku8MBSuccess) {
        actualPosition = (int32_t)((uint32_t)node.getResponseBuffer(1) << 16 | node.getResponseBuffer(0));
    } else {
        Serial.printf("MB Rd Fail Pos:0x%X\n", result);
        readSuccess = false;
    }

    if (!readSuccess) {
        modbusOk = false;
        if(servoIsEnabledTarget) { // Wenn Servo an sein sollte -> disable
             disableServoModbus();
        }
         servoIsEnabledTarget = false; // Ziel = AUS bei Lesefehler
         servoIsEnabledActual = false;
    }
    return readSuccess;
}

// --- WebSocket Event Handler ---
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WS Client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            // Sende initialen Status
            wsJsonTx.clear();
            wsJsonTx["type"] = "status";
            wsJsonTx["modbusOk"] = modbusOk;
            wsJsonTx["servoEnabled"] = servoIsEnabledActual;
            wsJsonTx["pos"] = actualPosition;
            wsJsonTx["spd"] = actualSpeed;
            wsJsonTx["trq"] = actualTorque;
            wsJsonTx["cur"] = rmsCurrent;
            wsJsonTx["vbus"] = busVoltage;
            { // Block für String-Erstellung
              String jsonString;
              serializeJson(wsJsonTx, jsonString);
              client->text(jsonString);
            }
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WS Client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA: {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                data[len] = 0; // Null-terminieren
                // Serial.printf("WS Rx from #%u: %s\n", client->id(), (char*)data); // Debug

                wsJsonRx.clear();
                DeserializationError error = deserializeJson(wsJsonRx, (char*)data);
                if (error) {
                    Serial.print(F("deserializeJson() failed: "));
                    Serial.println(error.f_str());
                    return;
                }

                const char* command = wsJsonRx["command"];
                if (command) {
                    if (strcmp(command, "setTorque") == 0) {
                        if (wsJsonRx.containsKey("value")) {
                            int16_t requestedTorque = wsJsonRx["value"];
                            // Begrenze auf sinnvollen Bereich (z.B. 0-1000 für 0-100.0%)
                            requestedTorque = constrain(requestedTorque, 0, 1000);
                            currentTargetTorque = requestedTorque; // Setze globales Ziel
                             // Sofort schreiben, wenn Servo aktiviert ist
                             if (servoIsEnabledActual && modbusOk) {
                                writeRegister(REG_TARGET_TORQUE, currentTargetTorque);
                             }
                            // Sende Bestätigung zurück (optional)
                            wsJsonTx.clear();
                            wsJsonTx["type"] = "torqueSet";
                            wsJsonTx["value"] = currentTargetTorque;
                             { // Block für String-Erstellung
                                String jsonString;
                                serializeJson(wsJsonTx, jsonString);
                                client->text(jsonString);
                             }
                        }
                    } else if (strcmp(command, "enableServo") == 0) {
                        servoIsEnabledTarget = true; // Setze Wunsch auf EIN
                         if (modbusOk) {
                            enableServoModbus(); // Versuche sofort zu aktivieren
                         } else {
                             Serial.println("Cannot enable: Modbus not OK");
                         }
                    } else if (strcmp(command, "disableServo") == 0) {
                        servoIsEnabledTarget = false; // Setze Wunsch auf AUS
                        // Setze auch Zieldrehmoment auf 0
                        currentTargetTorque = 0;
                        if (modbusOk) {
                            disableServoModbus(); // Versuche sofort zu deaktivieren
                            // Schreibe sicherheitshalber auch Torque 0
                            writeRegister(REG_TARGET_TORQUE, 0);
                        } else {
                             Serial.println("Cannot disable cleanly: Modbus not OK");
                             servoIsEnabledActual = false; // Zustand trotzdem auf AUS setzen
                        }
                    } else if (strcmp(command, "getStatus") == 0) {
                        // Client fragt explizit Status an (z.B. nach reconnect)
                         wsJsonTx.clear();
                         wsJsonTx["type"] = "status";
                         wsJsonTx["modbusOk"] = modbusOk;
                         wsJsonTx["servoEnabled"] = servoIsEnabledActual;
                         wsJsonTx["pos"] = actualPosition;
                         wsJsonTx["spd"] = actualSpeed;
                         wsJsonTx["trq"] = actualTorque;
                         wsJsonTx["cur"] = rmsCurrent;
                         wsJsonTx["vbus"] = busVoltage;
                         { // Block für String-Erstellung
                           String jsonString;
                           serializeJson(wsJsonTx, jsonString);
                           client->text(jsonString);
                         }
                    }
                }
            }
        } break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);
    Serial.println("\nModbus Servo Web Control Initializing (ESP32-S3)...");

    // --- WiFi Setup ---
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    int wifi_retries = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_retries < 20) {
        delay(500);
        Serial.print(".");
        wifi_retries++;
    }
    if (WiFi.status() != WL_CONNECTED) {
         Serial.println("\n!!! Failed to connect to WiFi !!!");
         // Hier könnte man einen Access Point Modus starten oder neu booten
         ESP.restart();
    } else {
        Serial.println("\nWiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }

    // --- Modbus Setup ---
    ModbusSerial.begin(115200, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
    if (!ModbusSerial) {
        Serial.println("!!! Failed to start Modbus Serial Port !!!");
         while (1) delay(100);
    } else {
        Serial.println("Modbus Serial Port OK.");
    }
    node.begin(SERVO_DRIVE_SLAVE_ID, ModbusSerial);

    Serial.println("Checking initial Modbus connection...");
    delay(500);
    checkModbusConnection(); // Erster Check
     if (!modbusOk) Serial.println("WARNING: Initial Modbus check failed!");

    // --- Drive Configuration ---
    Serial.println("Configuring Drive (if connection allows)...");
     if (modbusOk) {
        disableServoModbus(); // Sicherstellen, dass Servo AUS ist
        delay(100);
        if (!writeRegister(REG_CONTROL_MODE, 2)) Serial.println("Failed to set Control Mode!");
        delay(50);
        if (!writeRegister(REG_TORQUE_REF_SRC, 0)) Serial.println("Failed to set Torque Ref Source!");
        delay(50);
        if (!writeRegister(REG_TARGET_TORQUE, 0)) Serial.println("Failed to set initial Torque to 0!");
    } else {
        Serial.println("Skipping drive config due to connection issue.");
    }

    // --- Webserver & WebSocket Setup ---
    ws.onEvent(onWsEvent);        // WebSocket Event Handler registrieren
    server.addHandler(&ws);       // WebSocket Handler zum Server hinzufügen

    // Handler für die Hauptseite ("/")
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });

    // Webserver starten
    server.begin();
    Serial.println("HTTP server started. Open browser to ESP32 IP address.");

    // Initialisiere Timer
    lastModbusReadTime = millis();
    lastModbusCheckTime = millis();
    lastWsSendTime = millis();
    servoIsEnabledTarget = false; // Ziel ist initial AUS
    servoIsEnabledActual = false; // Faktisch initial AUS
}

// --- Loop ---
void loop() {
    unsigned long currentTime = millis();

    // 1. Modbus Verbindung prüfen (seltener)
    if (currentTime - lastModbusCheckTime >= modbusCheckInterval) {
        lastModbusCheckTime = currentTime;
        checkModbusConnection();
    }

    // 2. Modbus Daten lesen (häufig), nur wenn Verbindung OK
    if (modbusOk && (currentTime - lastModbusReadTime >= modbusReadInterval)) {
        lastModbusReadTime = currentTime;
        readServoData();
    }

    // 3. Servo Enable/Disable Logik (prüfen ob Ziel != Aktuell)
    if (modbusOk) {
        if (servoIsEnabledTarget && !servoIsEnabledActual) {
            // Soll AN sein, ist aber AUS -> Aktivieren versuchen
            enableServoModbus();
        } else if (!servoIsEnabledTarget && servoIsEnabledActual) {
            // Soll AUS sein, ist aber AN -> Deaktivieren versuchen
            disableServoModbus();
        }
        // Schreibe Zieldrehmoment, wenn aktiviert und Wert gesetzt
         if (servoIsEnabledActual && currentTargetTorque >= 0) { // >= 0 nur als Beispiel
             // Nur schreiben, wenn es notwendig ist (wird durch Slider-Event/Disable getriggert)
             // Optional: Periodisches Schreiben hinzufügen, falls notwendig
             // writeRegister(REG_TARGET_TORQUE, currentTargetTorque);
         }
    } else {
        // Modbus nicht OK
        if (servoIsEnabledActual) { // Wenn er AN war -> Zustand korrigieren
            servoIsEnabledActual = false;
            Serial.println("Loop: Modbus lost, setting actual state to OFF.");
        }
         servoIsEnabledTarget = false; // Ziel auch auf AUS
    }


    // 4. Daten an WebSocket Clients senden (weniger häufig als Modbus lesen)
    if (currentTime - lastWsSendTime >= wsSendInterval) {
        lastWsSendTime = currentTime;
        if (ws.count() > 0) { // Nur senden, wenn Clients verbunden sind
            wsJsonTx.clear();
            wsJsonTx["type"] = "status";
            wsJsonTx["modbusOk"] = modbusOk;
            wsJsonTx["servoEnabled"] = servoIsEnabledActual; // Sende den tatsächlichen Zustand
            wsJsonTx["pos"] = actualPosition;
            wsJsonTx["spd"] = actualSpeed;
            wsJsonTx["trq"] = actualTorque;
            wsJsonTx["cur"] = rmsCurrent;
            wsJsonTx["vbus"] = busVoltage;
            { // Block für String-Erstellung
              String jsonString;
              serializeJson(wsJsonTx, jsonString);
              ws.textAll(jsonString); // An alle verbundenen Clients senden
            }
        }
    }

    // WebSocket Aufräumarbeiten (wichtig für AsyncWebServer)
    ws.cleanupClients();

    delay(1); // Kleine Pause für den Scheduler
}
