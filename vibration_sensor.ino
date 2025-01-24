#include <Servo.h>
#include "DHT.h"

// Servo configuration
Servo s1;

// Pin definitions
int sensor_pin = A0;
int led_pin = 7; // Pin for LED
int buzzer_pin = 8; // Pin for Buzzer

// DHT22 configuration
#define DHTPIN 2 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // Define the type of DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Timers
unsigned long previousMillisServo = 0;
unsigned long previousMillisSensor = 0;
unsigned long previousMillisBuzzer = 0;
unsigned long previousMillisCycle = 0;
unsigned long previousMillisDHT = 0; // Timer for DHT sensor readings

const unsigned long servoInterval = 500;   // Servo oscillation delay
const unsigned long sensorInterval = 1000; // Moisture check interval
const unsigned long buzzerInterval = 1000; // Buzzer and LED active duration
const unsigned long activeDuration = 20000; // Active phase duration
const unsigned long idleDuration = 10000;  // Idle phase duration
const unsigned long dhtInterval = 2000;    // DHT reading interval (every 2 seconds)

bool isActivePhase = true;
bool servoDirection = true; // To toggle servo direction
bool buzzerActive = false;

// Additional code pin definitions
int porta_a0 = A2;  // Analog pin for additional sensor
int porta_D0 = 4;   // Digital pin for additional sensor
int leitura_porta_analogica = 0;
int leitura_porta_digital = 0;
int pinoled = 7; // Reuse led_pin for consistency

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Attach the servo to pin 9
  s1.attach(9);

  // Set pin modes
  pinMode(sensor_pin, INPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);

  // Set pin modes for additional sensor
  pinMode(porta_a0, INPUT);
  pinMode(porta_D0, INPUT);
  pinMode(pinoled, OUTPUT);

  // Initialize DHT sensor
  dht.begin();

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

  // Read temperature and humidity from the DHT22 sensor
  if (currentMillis - previousMillisDHT >= dhtInterval) {
    previousMillisDHT = currentMillis;

    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperature();
    float temperatureF = dht.readTemperature(true);

    // Check if any reads failed
    if (isnan(humidity) || isnan(temperatureC) || isnan(temperatureF)) {
      Serial.println(F("Failed to read from DHT sensor!"));
    } else {
      // Print DHT sensor readings
      Serial.print(F("Humidity: "));
      Serial.print(humidity);
      Serial.print(F("%  Temperature: "));
      Serial.print(temperatureC);
      Serial.print(F("\u00B0C "));
      Serial.print(temperatureF);
      Serial.println(F("\u00B0F"));
    }
  }

  // Additional sensor operations
  leitura_porta_analogica = analogRead(porta_a0);
  leitura_porta_digital = digitalRead(porta_D0);

  // Print values from additional sensor
  Serial.print("Porta Digital : ");
  Serial.print(leitura_porta_digital);
  Serial.print(" Porta Analogica : ");
  Serial.println(leitura_porta_analogica);

  // Test if additional sensor was triggered and toggle LED
  if (leitura_porta_digital != 1) {
    digitalWrite(pinoled, HIGH);
    delay(10);
    digitalWrite(pinoled, LOW);
  }

  delay(100);
}
