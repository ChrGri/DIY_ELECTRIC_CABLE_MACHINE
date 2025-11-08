# ESP32-Powered DIY Smart Cable Machine

An open-source, smart digital resistance machine using an ESP32, a high-torque StepperOnline A6 servo, and 3D-printed components. 

<img width="748" height="993" alt="image" src="https://github.com/user-attachments/assets/911381d6-aee3-4775-b78f-ad645e522a1c" />

<img width="1319" height="993" alt="image" src="https://github.com/user-attachments/assets/f03fdc0b-d693-4f28-ae2a-7654bda9b814" />

<img width="1319" height="993" alt="image" src="Doc/Images/First_Usage.gif" />

Action video, see [here](Doc/Videos/ActionVideo_1.mp4).


The machine can be parameterized via web interface see <br>
<img width="800" alt="image" src="Doc/Images/WebinterfaceDemo1.gif" />

# 🏋️ About The Project

This project is a Do-It-Yourself (DIY) approach to modern smart gyms. Instead of using a traditional weight stack, this machine uses a powerful closed-loop servo motor (StepperOnline A6) controlled by an ESP32 to provide resistance. This allows for digitally adjustable weight, smooth operation, and the potential for advanced features like eccentric-only training, custom resistance curves, and workout tracking.

The core of the build relies on accessible components: the powerful ESP32, a reliable industrial servo, a standard gym cable/handle, and custom 3D-printed parts for the spool and mounting. The ESP32 S3 was selected, since it has onboard WiFi and Bluetooth functionality and sufficient compute to allow web based control.  

## ✨ Key Features

- Digitally Adjustable Resistance: Change your "weight" from a web interface or physical buttons.
- High-Torque Servo: The StepperOnline A6 servo provides strong, smooth, and precise resistance.
- ESP32 Controller: Wi-Fi and Bluetooth-ready for a new UI, mobile app control, and OTA (Over-The-Air) updates.
- Fully Open-Source: All 3D models (STL/STEP) and firmware code are provided.
- Cost-Effective: Built for a fraction of the cost of commercial smart gyms.
- Customizable: Modify the code to add new modes (eccentric, isometric, chains) or integrate with other fitness apps.
- controller via web interface --> 
<img width="300" alt="image" src="https://github.com/user-attachments/assets/b3decf32-4f61-4d17-a9bc-2b21e9cfe277" />




# Support the team
I :heart: doing research. New hardware (e.g. servos, periphery, PCBs) is very expensive. Feel free to support me and thus fasten up the research activity.

Dev | captainchris |
--- | --- |
Buy me a coffee | <a href="https://www.buymeacoffee.com/Captainchris"><img src="https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png" height="20px"></a> | 
Ko-fi | [![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/captainchris88) |

## 🛠️ Bill of Materials (BOM)

Core Components
| Component | Model / Spec | Notes | (affiliate) link|
|--|--|--|--|
| Microcontroller | ESP32 S3 dev board |  | [Amazon](https://amzn.to/48NTnuW) |
| RJ45  cable |  | | [Amazon](https://amzn.to/47rE3Si) |
| TTL to Modbus converter |  |  | [Amazon](https://amzn.to/4nr72vo) |
| Servo Motor | StepperOnline A6 Servo | e.g., A6-750W or similar. Choose based on desired power. | [A6 1,27Nm](https://www.omc-stepperonline.com/de/a6-serie-400w-rs485-ac-servomotor-kit-3000rpm-1-27nm-17-bit-absolutwertgeber-ip67-a6-rs400h2a1-m17?tracking=6721c5865911c)
| Gym Cable | 3mm Polyster cord x 30m | Polyster cord is strong. | [Polyster cord](https://amzn.to/49fXEY2) |
| Handle | Standard D-Handle | Any cable machine attachment works. |  |

### Custom PCB
In the making.

### 3D Printed Parts

- Main Spool (attaches to servo shaft)
- Motor Mount
- ESP32 Enclosure

(All .stl and .step files are available in the /3D-Models directory of this repository)

## 🔌 Wiring Diagram

Here is the basic wiring schematic for connecting the ESP32 to the servo driver.

Warning: You are working with high-voltage AC (for the PSU) and high-current DC (for the servo). Always double-check your connections and ensure everything is unplugged when wiring.

### Connect the ESP32 to the TTL-to-Modbus board
   
| from ESP32 <br> <img width="300" alt="image" src="https://github.com/user-attachments/assets/565de791-3d64-4211-a45e-7528fca7ca63" /> | TTL-to-Modbus Adapter <br> <img width="300" alt="image" src="https://github.com/user-attachments/assets/a77039a2-16b3-4150-8030-5f5a5e6e795c" /> |
|--|--|
| 3V3 | VCC |
| GPIO 4 (TX) | TXD |
| GPIO 6 (RX)  | RXD | 
| GND | GND |

### Connect the TTL-to-Modbus board to the RJ45 plug
| TTL-RJ45 Adapter <br> <img width="300" alt="image" src="https://github.com/user-attachments/assets/a77039a2-16b3-4150-8030-5f5a5e6e795c" /> | RJ45 <br> <img width="300" alt="image" src="https://github.com/user-attachments/assets/48953fcf-1dca-4519-84ba-6b3fe31a07a5" /> |
|--|--|
| A+  | Pin 4 | 
| B- | Pin 5 |
| GND | Pin 8 |

<img width="300" alt="image" src="https://github.com/user-attachments/assets/7676d3fb-c37f-4bac-9296-f0dc9edf521c" /> <br>
For Modbus communication and wiring please refer to this https://help.stepperonline.com/en/article/a6-servo-motor-rs485-operation-instruction-1u47bbl/

In the end, the wiring should look like this <br>
<img width="1920" alt="image" src="https://github.com/user-attachments/assets/7debd940-cc97-4287-b648-e6949aee0342" />.

### Connect the RJ45 plug into the A6 servos CN3 port. 
Connect the RJ45 plug to CN3 port of the servo, port (5) in the image below:<br> 
<img width="300" alt="image" src="https://github.com/user-attachments/assets/6a706332-f489-4afa-a6c6-19f2577885f1" />

### AC and Servo wiring
Please refer to the A6 manual for wiring of the A6 servo. As an example, here is how mine is wired up: <br>
<img width="800" alt="image" src="https://github.com/user-attachments/assets/1e850616-a45b-45bc-85a9-74cca3e09376" />



## 💾 Firmware Setup

The firmware is written using the Arduino framework for the ESP32.
### VSCode
Install [VSCode](https://code.visualstudio.com/)

### PlatformIO
Install [PlatformIO](https://platformio.org/install/ide?install=vscode) in Visual Studio Code

> ![Note]
> When you restart VS Code for the first time after the PlatformIO extension is installed, it will install and build the development environment which will take a moment. Wait until it is complete before attempting to open the project.

### Flashing the firmware

1. IDE: Open [Code/Esp32S3](https://github.com/ChrGri/DIY_ELECTRIC_CABLE_MACHINE/tree/main/Code/Esp32S3) folder in VSCode
2. Flash: Build and upload the firmware to your ESP32.

## 🚀 How to Use

1. Power On: Connect the power supply and turn on the system.
2. Connect: The ESP32 will create a Wi-Fi network (or join yours, based on your config). Connect to its IP address from your phone or computer.
3. Web Interface: You will see a simple web page.
4. Set Weight: Enter your desired resistance (in kg or lbs) and press "Set".
5. Work Out! The servo will engage and provide resistance as you pull the handle.

## 📈 To-Do / Future Plans

[ ] Read servo alarms

[ ] Add eccentric resistance mode (higher resistance on release).

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
