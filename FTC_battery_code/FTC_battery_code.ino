//LOAD DRAWS 25 AMPS FORM THE LOAD
/*| Arduino Pin | Connects To                            |
| ----------- | -------------------------------------- |
| Pin 2       | Button → Other side of button to GND   |
| Pin 4       | Relay IN1 (controls positive charger)  |
| Pin 5       | Relay IN2 (controls negative charger)  |
| Pin 6       | Relay IN3 (controls positive load)     |
| Pin 7       | Relay IN4 (controls negative load)     |
| A0          | Middle of voltage divider from battery |
*/

// Relay Control Pins 
const int positive_charger = 4;
const int negative_charger = 5;
const int positive_load = 6;
const int negative_load = 7;
const int buttonPin = 2;
const int voltagePin = A0;

// Voltage Divider Resistor Values
const float R1 = 2157.0;  // 2.12k ohms (from battery + to A0)
const float R2 = 984.0;   // 0.977k ohms (from A0 to GND)

// Calculate voltage divider factor:
const float voltageDividerFactor = 3.358; //(R1 + R2) / R2; //=3.2

int raw = 0;
float dividedVoltage = 0.0;
float voltage = 0.0;

// Voltage thresholds 
const float chargedThreshold = 12.3;
const float lowVoltageThresholdPb = 9.5;
const float lowVoltageThresholdNiMh = 8.5;
const float lowVoltageThreshold = lowVoltageThresholdNiMh;

void setup() {
  Serial.begin(9600);               // Start serial for debugging
  analogReadResolution(10);  
  pinMode(positive_charger, OUTPUT);
  pinMode(negative_charger, OUTPUT);
  pinMode(positive_load, OUTPUT);
  pinMode(negative_load, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);  // Button with internal pull-up resistor

  allOff();                         // Turn off all relays at start

  Serial.println("System Powered On. Reset Ah meter. Press button to start.");
}

void loop() {
  // wait for button to be pressed
  //turn charger off, load on
  //loop waiting for the battery to be discharged
  //after loop is done, turn load off charger on, print how long test took

  // Wait for button to be pressed
  Serial.println("Start Up");
  while (digitalRead(buttonPin) == HIGH) {
    delay(10); 
  }
  
  Serial.println("Button pressed! Starting discharge test...");
  Serial.print("Starting voltage (no-load): ");
  printVoltage();
  Serial.println("");
  
  // Record start time
  unsigned long startTime = millis();
  
  // Turn charger off, load on
  powerLoad();
  Serial.println("Load activated");
  
  // Loop waiting for the battery to be discharged
  Serial.println("Monitoring battery discharge...");
  while (true) {
    printElapsedTime(startTime);
    Serial.print(", ");
    printVoltage();
    Serial.println("");
    
    // Check if battery is discharged to threshold
    if (voltage <= lowVoltageThreshold) {
      Serial.println("Battery discharged to threshold!");
      break;
    }
    
    delay(1000); // Checks voltage every 1 second
  }
  
  // After loop is done, turn load off charger on, print how long test took
  unsigned long endTime = millis();
  unsigned long testDuration = endTime - startTime;
  
  allOff(); // Turn off load
  delay(1000); 
  
  Serial.println("DISCHARGE COMPLETE");
  Serial.print("Test duration: ");
  printElapsedTime(startTime);
  Serial.println ("");
  Serial.println("Press button to start next test");
  Serial.println("===============================");
}

void printVoltage(){
  readBatteryVoltage();
  // Serial.print("Raw Measure: ");
  // Serial.print(raw);
  // Serial.print(", Analog Measure: ");
  // Serial.print(dividedVoltage);
  // Serial.print(", ");
  Serial.print("Battery voltage: ");
  Serial.print(voltage);
}

void printElapsedTime(unsigned long startTime) {
  unsigned long duration = millis() - startTime;
  Serial.print(duration / (60*1000));
  Serial.print("m");
  Serial.print((duration % (60*1000))/1000);
  Serial.print("s");
}
void allOff() {
  digitalWrite(positive_charger, HIGH);
  digitalWrite(negative_charger, HIGH);
  digitalWrite(positive_load, HIGH);
  digitalWrite(negative_load, HIGH);
}

void powerLoad() {
  digitalWrite(positive_charger, HIGH);
  digitalWrite(negative_charger, HIGH);
  digitalWrite(positive_load, LOW);
  digitalWrite(negative_load, LOW);
}


void readBatteryVoltage() {
  raw = analogRead(voltagePin); //10 bit binary number 0=0volts, 1023= 5volts
  dividedVoltage = raw * (5.0 / 1023.0) * 0.945; //converts that into 0.0 = 0volts and 5.0 =5volts calibration added to the adc to calibrate this
  voltage = dividedVoltage * voltageDividerFactor;
}
