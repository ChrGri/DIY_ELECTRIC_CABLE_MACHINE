# ESP32-Powered DIY Smart Cable Machine

An open-source, smart digital resistance machine using an ESP32, a high-torque StepperOnline A6 servo, and 3D-printed components. Ditch the iron plates and build your own Tonal/Vitruvian-style trainer!

<img width="748" height="993" alt="image" src="https://github.com/user-attachments/assets/911381d6-aee3-4775-b78f-ad645e522a1c" />

# 🏋️ About The Project

This project is a Do-It-Yourself (DIY) approach to modern smart gyms. Instead of using a traditional weight stack, this machine uses a powerful closed-loop servo motor (StepperOnline A6) controlled by an ESP32 to provide resistance. This allows for digitally adjustable weight, smooth operation, and the potential for advanced features like eccentric-only training, custom resistance curves, and workout tracking.

The core of the build relies on accessible components: the powerful ESP32, a reliable industrial servo, a standard gym cable/handle, and custom 3D-printed parts for the spool and mounting.

## ✨ Key Features

- Digitally Adjustable Resistance: Change your "weight" from a web interface or physical buttons.
- High-Torque Servo: The StepperOnline A6 servo provides strong, smooth, and precise resistance.
- ESP32 Controller: Wi-Fi and Bluetooth-ready for a new UI, mobile app control, and OTA (Over-The-Air) updates.
- Fully Open-Source: All 3D models (STL/STEP) and firmware code are provided.
- Cost-Effective: Built for a fraction of the cost of commercial smart gyms.
- Customizable: Modify the code to add new modes (eccentric, isometric, chains) or integrate with other fitness apps.

## 🛠️ Bill of Materials (BOM)

Core Components
| Component | Model / Spec | Notes |
|--|--|--|
| Microcontroller | ESP32 Dev Board | Any ESP32-WROOM-32 board will work. |
| Servo Motor | StepperOnline A6 Servo | e.g., A6-750W or similar. Choose based on desired power. | 
| Servo Driver | Matching A6 Driver | This typically comes bundled with the motor. | 
| Power Supply | Integrated 230V AC | |
| Gym Cable |3mm-5mm Steel or Dyneema | Dyneema cord is strong and flexible. |
| Handle | Standard D-Handle | Any cable machine attachment works. | 


### 3D Printed Parts

- Main Spool (attaches to servo shaft)
- Motor Mount
- ESP32 Enclosure

(All .stl and .step files are available in the /3D-Models directory of this repository)

## 🔌 Wiring Diagram

Here is the basic wiring schematic for connecting the ESP32 to the servo driver.

Warning: You are working with high-voltage AC (for the PSU) and high-current DC (for the servo). Always double-check your connections and ensure everything is unplugged when wiring.

(A simplified diagram. The A6 driver manual will have detailed pinouts for alarm, enable, and pulse/dir signals.)

                 +-------------------+
                 |  StepperOnline A6 |
(from ESP32)     |   Servo Driver    |
                 |                   |
GPIO (Pulse) ---->| PUL+              |
GPIO (Dir)   ---->| DIR+              |
GPIO (Enable) --->| ENA+              |
GND          ---->| PUL- / DIR- / ENA-|
                 |                   |
                 |     MOTOR (M)     |----> (To A6 Servo Motor)
                 |     POWER (L1/L2) |----> (To 48V DC PSU)
                 +-------------------+


## 💾 Firmware Setup

The firmware is written using the Arduino framework for the ESP32.

1. IDE: Open this project in PlatformIO (recommended) or the Arduino IDE.
2. Libraries: Install the required libraries (e.g., ESPAsyncWebServer, AccelStepper).
3. Flash: Build and upload the firmware to your ESP32.

## 🚀 How to Use

1. Power On: Connect the power supply and turn on the system.
2. Connect: The ESP32 will create a Wi-Fi network (or join yours, based on your config). Connect to its IP address from your phone or computer.
3. Web Interface: You will see a simple web page.
4. Set Weight: Enter your desired resistance (in kg or lbs) and press "Set".
5. Work Out! The servo will engage and provide resistance as you pull the handle.

## 📈 To-Do / Future Plans

[ ] Add eccentric resistance mode (higher resistance on release).

[ ] Implement a BLE (Bluetooth Low Energy) service for a mobile app.

[ ] Add a load cell to measure force accurately and provide real-time feedback.

[ ] Workout tracking and history.

[ ] Add physical buttons/encoder to set weight without a phone.

## 🤝 Contributing

Pull requests are welcome! If you have ideas for improvements, new features, or bug fixes, please open an issue first to discuss what you would like to change.

1. Fork the Project
2. Create your Feature Branch (git checkout -b feature/AmazingFeature)
3. Commit your Changes (git commit -m 'Add some AmazingFeature')
4. Push to the Branch (git push origin feature/AmazingFeature)
5. Open a Pull Request

## ⚠️ Safety Disclaimer ⚠️

Build and use this device at your own risk.

This is an advanced DIY project that involves:

- High-Voltage AC: You will be working with mains AC voltage for the power supply, which carries a risk of severe electric shock or death.
- Powerful Motor: The A6 servo motor is extremely powerful. It can cause serious injury if hands, hair, or clothing get caught in the spool or cable.
- High Tension: The machine operates under high tension. Failure of 3D-printed parts, the cable, or mountings can result in a sudden release of load, posing a significant risk of injury.

The author(s) and contributors to this project assume no responsibility or liability for any injuries, damages, or losses resulting from the construction or use of this device. You are solely responsible for ensuring the safety of your build, performing all necessary safety checks, and using the machine in a safe manner.

## 📄 License
This project is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0). See the LICENSE file for details.
