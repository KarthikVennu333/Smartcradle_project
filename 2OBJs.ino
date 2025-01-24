#include <Servo.h>

Servo s1;

int sensor_pin = A0;
int led_pin = 7; // Pin for LED
int buzzer_pin = 8; // Pin for Buzzer

// Timers
unsigned long previousMillisServo = 0;
unsigned long previousMillisSensor = 0;
unsigned long previousMillisBuzzer = 0;
unsigned long previousMillisCycle = 0;

const unsigned long servoInterval = 500;   // Servo oscillation delay
const unsigned long sensorInterval = 1000; // Moisture check interval
const unsigned long buzzerInterval = 1000; // Buzzer and LED active duration
const unsigned long activeDuration = 20000; // Active phase duration
const unsigned long idleDuration = 10000;  // Idle phase duration

bool isActivePhase = true;
bool servoDirection = true; // To toggle servo direction
bool buzzerActive = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Attach the servo to pin 9
  s1.attach(9);

  // Set pin modes
  pinMode(sensor_pin, INPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);

  s1.write(0); // Initialize servo to 0 degrees
}

void loop() {
  unsigned long currentMillis = millis();

  // Switch between active and idle phases
  if (isActivePhase && currentMillis - previousMillisCycle >= activeDuration) {
    isActivePhase = false;
    previousMillisCycle = currentMillis;
    s1.write(0); // Stop servo in idle phase
  } else if (!isActivePhase && currentMillis - previousMillisCycle >= idleDuration) {
    isActivePhase = true;
    previousMillisCycle = currentMillis;
  }

  // Handle servo motor during active phase
  if (isActivePhase && currentMillis - previousMillisServo >= servoInterval) {
    previousMillisServo = currentMillis;
    if (servoDirection) {
      s1.write(0); // Move to 0 degrees
    } else {
      s1.write(180); // Move to 180 degrees
    }
    servoDirection = !servoDirection; // Toggle direction
  }

  // Continuous moisture detection
  if (currentMillis - previousMillisSensor >= sensorInterval) {
    previousMillisSensor = currentMillis;

    int sensor_data = analogRead(sensor_pin);
    Serial.println(sensor_data);

    if (sensor_data >= 100 && sensor_data <= 1000) {
      Serial.println("Wetness Detected... Change the diaper");
      buzzerActive = true;
      previousMillisBuzzer = currentMillis; // Start buzzer timer
    }
  }

  // Handle buzzer and LED when wetness is detected
  if (buzzerActive) {
    if (currentMillis - previousMillisBuzzer <= buzzerInterval) {
      digitalWrite(led_pin, HIGH);
      digitalWrite(buzzer_pin, HIGH);
    } else {
      digitalWrite(led_pin, LOW);
      digitalWrite(buzzer_pin, LOW);
      buzzerActive = false; // Stop the buzzer after duration
    }
  }
}
