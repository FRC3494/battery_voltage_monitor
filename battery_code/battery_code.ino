
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
const float voltageDividerFactor = 3.189;//(R1 + R2) / R2; //=3.2

int raw = 0;
float dividedVoltage = 0.0;

// Voltage thresholds 
const float chargedThreshold = 12.3;
const float lowVoltageThreshold = 9.5;

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

  // Wait until button is pressed (buttonPin LOW)
  // while (digitalRead(buttonPin) == HIGH) { // should be at the top of loop, shld start the loop !!!!!!!

 // analogReadResolution(10); //change to 10-bit resolution necessary for the uno r4 but not supported by the old uno
 // analogReference(AR_DEFAULT); //Set Analog Reference to 5V
}

void loop(){
 // wait for button to be pressed
  //turn charger off, load on
  //loop waiting for the battery to be discharged
  //after loop is done, turn load off charger on, print how long test took

  // Wait for button to be pressed
  Serial.print("Start Up");
  while (digitalRead(buttonPin) == HIGH) {
    delay(10); 
  }
  
  Serial.println("Button pressed! Starting discharge test...");
  
  // Record start time
  unsigned long startTime = millis();
  
  // Turn charger off, load on
  powerLoad();
  Serial.println("Load activated, charger deactivated");
  
  // Loop waiting for the battery to be discharged
  Serial.println("Monitoring battery discharge...");
  while (true) {
    float voltage = readBatteryVoltage();
     Serial.print("Raw Measure: ");
    Serial.println(raw);
    Serial.print("Analog Measure: ");
    Serial.println(dividedVoltage);
    Serial.print("Current voltage: ");
    Serial.println(voltage);
    
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
  chargeBattery(); // Turn on charger
  
  Serial.println("DISCHARGE COMPLETE");
  Serial.print("Test duration: ");
  /*Serial.print(testDuration / 1000.0);
  Serial.println(" seconds");
  Serial.print("Test duration: ");
  Serial.print(testDuration / 60000.0); // Convert to minutes
  Serial.println(" minutes");*/
  Serial.print (testDuration / (60*1000));
  Serial.print(":");
  Serial.print (testDuration % (60*1000));
  Serial.println("minutes");
  Serial.println("Battery is now charging...");
  Serial.println("Press button to start next test");
  Serial.println("===============================");
}

void chatgptloop() {
  float voltage = readBatteryVoltage();
  Serial.print("Battery Voltage: ");
  Serial.println(voltage);

  if (voltage > chargedThreshold) {
    Serial.println("Battery charged. Powering load...");
    powerLoad();

    delay(2000);  // Let voltage settle after turning on load

    voltage = readBatteryVoltage();
    Serial.print("Voltage under load: ");
    Serial.println(voltage);

    if (voltage < lowVoltageThreshold) {                      // new loop: keeps going on until the voltage is below the min voltage
      shutdownAll("Voltage dropped too low under load");
      return;
    }

    Serial.println("Entering 'wait and see' mode...");
    delay(3000);  // Wait and observe voltage

    voltage = readBatteryVoltage();
    Serial.print("Observed voltage: ");
    Serial.println(voltage);

    if (voltage < lowVoltageThreshold) {
      shutdownAll("Voltage too low after wait");
      return;
    }

    Serial.println("Battery holding up. Ending cycle safely.");
    shutdownAll("Ending cycle.");
    return;

  } else {
    Serial.println("Battery not charged enough. Waiting and charging.");
    allOff();
  }
}

void allOff() {
  digitalWrite(positive_charger, LOW);
  digitalWrite(negative_charger, LOW);
  digitalWrite(positive_load, LOW);
  digitalWrite(negative_load, LOW);
}

void powerLoad() {
  digitalWrite(positive_charger, LOW);
  digitalWrite(negative_charger, LOW);
  digitalWrite(positive_load, HIGH);
  digitalWrite(negative_load, HIGH);
}

void chargeBattery() {
  digitalWrite(negative_load, LOW);
  digitalWrite(positive_load, LOW);
  digitalWrite(positive_charger, HIGH);
  digitalWrite(negative_charger, HIGH);
  
}

void shutdownAll(const char* reason) {
  Serial.println(reason);
  allOff();
  delay(30000);  // Wait 30 seconds before charging
  chargeBattery();
  Serial.println("Charging battery now..."); //FIX - 
}


float readBatteryVoltage() {
  raw = analogRead(voltagePin); //10 bit binary number 0=0volts, 1023= 5volts
  dividedVoltage = raw * (5.0 / 1023.0)*0.945; //converts that into 0.0 = 0volts and 5.0 =5volts calibration added to the adc to calibrate this
  return dividedVoltage * voltageDividerFactor;
}
