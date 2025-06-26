// === Relay Control Pins ===
const int positive_charger = 4;
const int negative_charger = 5;
const int positive_load = 6;
const int negative_load = 7;
const int buttonPin = 2;
const int voltagePin = A0;

// === Voltage Divider Resistor Values ===
// Replace these with your actual resistor values in ohms
const float R1 = 30000.0;  // e.g., 30k ohms (from battery + to A0)
const float R2 = 7500.0;   // e.g., 7.5k ohms (from A0 to GND)

// Calculate voltage divider factor:
// voltageDividerFactor = (R1 + R2) / R2
const float voltageDividerFactor = (R1 + R2) / R2;

// === Voltage thresholds ===
const float chargedThreshold = 12.3;
const float lowVoltageThreshold = 9.5;

void setup() {
  Serial.begin(9600);               // Start serial for debugging

  pinMode(positive_charger, OUTPUT);
  pinMode(negative_charger, OUTPUT);
  pinMode(positive_load, OUTPUT);
  pinMode(negative_load, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);  // Button with internal pull-up resistor

  allOff();                         // Turn off all relays at start

  Serial.println("System Powered On. Reset Ah meter. Press button to start.");

  // Wait until button is pressed (buttonPin LOW)
  while (digitalRead(buttonPin) == HIGH) {
    delay(100);
  }
}

void loop() {
  float voltage = readBatteryVoltage();
  Serial.print("Initial Voltage: ");
  Serial.println(voltage);

  if (voltage > chargedThreshold) {
    Serial.println("Battery charged. Powering load...");
    powerLoad();

    delay(2000);  // Let voltage settle after turning on load

    voltage = readBatteryVoltage();
    Serial.print("Voltage under load: ");
    Serial.println(voltage);

    if (voltage < lowVoltageThreshold) {
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
  digitalWrite(positive_charger, HIGH);
  digitalWrite(negative_charger, HIGH);
  digitalWrite(positive_load, LOW);
  digitalWrite(negative_load, LOW);
}

void shutdownAll(const char* reason) {
  Serial.println(reason);
  allOff();
  delay(30000);  // Wait 30 seconds before charging
  chargeBattery();
  Serial.println("Charging battery now...");
}

float readBatteryVoltage() {
  int raw = analogRead(voltagePin);
  float voltage = raw * (5.0 / 1023.0) * voltageDividerFactor;
  return voltage;
}
