// C03.40 = 1 AI1 analog reference


#include <driver/dac.h>

// --- New ---
// Define the ADC pin for the potentiometer
// GPIO 34 is a good choice as it's ADC1_CH6 and input-only
const int potPin = 34; 
// --- End New ---


// // Pins for LOLIN S2 Mini
// const int dacPin = 17; // DAC Channel 1 is GPIO17 on ESP32-S2
// const dac_channel_t dacChannel = DAC_CHANNEL_1; // Use DAC_CHANNEL_1 enum

const int dacPin = 25; // DAC Channel 1 is GPIO25 on standard ESP32
const dac_channel_t dacChannel = DAC_CHANNEL_1; // Use DAC_CHANNEL_1 enum

const int servoOnPin = 4; // Using GPIO4 for S-ON
#define PI_FL32         3.1415926535897932384626433832795f

void setup() {
  Serial.begin(115200);
  pinMode(servoOnPin, OUTPUT);
  digitalWrite(servoOnPin, HIGH); // Start with Servo OFF (S-ON inactive)

  // Configure DAC
  dac_output_enable(dacChannel);
  dac_output_voltage(dacChannel, 0); // Set initial torque reference to 0V (DAC value 0)

  // --- New ---
  // Set the potentiometer pin as an input
  pinMode(potPin, INPUT);
  Serial.println("ADC and DAC Setup Complete. Potentiometer connected to GPIO 34?");
  // --- End New ---

  delay(2000); // Wait a bit

  // --- Example: Enable Servo and command ~1V (approx 10% torque if 10V=100%) ---
  // Note: Your comment says "S-ON HIGH" but code sets it LOW to activate
  Serial.println("Enabling Servo (S-ON LOW)"); 
  digitalWrite(servoOnPin, LOW); // Activate S-ON
  delay(500); // Allow time for drive to enable
}

void loop() {

  // --- Replaced Sine Wave Calculation ---
  // long timeNowMillis = millis();
  // float seconds_fl32 = timeNowMillis / 1000.0f;
  // float f_Hz_fl32 = 0.2f;

  // float amplitude = 100.0f * fabsf( sinf( 2.0f * PI_FL32 * f_Hz_fl32 * seconds_fl32 ) );

  // amplitude = constrain(amplitude, 0.0f, 255.0f);



  // 1. Read the potentiometer value. 
  // On ESP32, analogRead() returns a 12-bit value (0 to 4095)
  int potValue = analogRead(potPin);

  // 2. Map the 12-bit ADC range (0-4095) to the 8-bit DAC range (0-255)
  int dacValue = map(potValue, 0, 4095, 0, 255);
  
  // 3. Constrain the value just in case of ADC noise (optional but good practice)
  dacValue = constrain(dacValue, 0, 255);
  
  // --- End of Replaced Section ---


  // // --- Example: Enable Servo and command ~1V (approx 10% torque if 10V=100%) ---
  // Serial.println("Enabling Servo (S-ON HIGH)");
  // digitalWrite(servoOnPin, LOW); // Activate S-ON
  // delay(500); // Allow time for drive to enable

  // Calculate DAC value for desired voltage (approx 1V)
  // DAC is 8-bit (0-255), output is ~0V to 3.3V
  // int dacValue = (int)((1.0 / 3.3) * 255);
  
  // This value is now controlled by the potentiometer mapping above
  // int dacValue = (int)amplitude; 

  // --- Modified Serial Print ---
  Serial.print("Pot (ADC): ");
  Serial.print(potValue);
  Serial.print(" -> Setting DAC output: ");
  Serial.print(dacValue);
  Serial.println(")");
  // --- End Modified Serial Print ---

  dac_output_voltage(dacChannel, dacValue);

  delay(50); // Poll the potentiometer every 50ms

  // // --- Example: Ramp down torque and disable servo ---
  // Serial.println("Ramping down torque");
  // for (int val = dacValue; val >= 0; val--) {
  //   dac_output_voltage(dacChannel, val);
  //   delay(10);
  // }

  // Serial.println("Disabling Servo (S-ON LOW)");
  // digitalWrite(servoOnPin, HIGH); // Deactivate S-ON
  // delay(1000); // Wait

  // // --- Loop break or add other commands ---
  // while (true) { // Stop after one cycle for this example
  //    delay(1000);
  // }

  // error and alarm codes on page 213
}