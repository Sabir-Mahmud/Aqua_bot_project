#include <Servo.h>
#include <SoftwareSerial.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// Define SoftwareSerial pins
#define RX_PIN 2 // Arduino RX pin (connect to TX of Bluetooth module)
#define TX_PIN 3 // Arduino TX pin (connect to RX of Bluetooth module)

// Create SoftwareSerial object
SoftwareSerial bluetooth(RX_PIN, TX_PIN);

// Define motor control pins
#define LEFT_MOTOR_FORWARD 6
#define LEFT_MOTOR_BACKWARD 5
#define RIGHT_MOTOR_FORWARD 11
#define RIGHT_MOTOR_BACKWARD 10

#define RIGHT_FACT 0.8

// Servo pin for door
#define SERVO_PIN 9
#define pump      8
#define SONAR     7

// Create servo object
Servo doorServo;

// Door positions
const int DOOR_OPEN_ANGLE = 180;  // Open position
const int DOOR_CLOSED_ANGLE = 0; // Closed position

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  bluetooth.begin(9600);

  // Initialize motor control pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  pinMode(pump, OUTPUT);
  pinMode(SONAR, OUTPUT);

  // Initialize servo
  doorServo.attach(SERVO_PIN);
  doorServo.write(DOOR_CLOSED_ANGLE); // Ensure door starts closed

    sensors.begin();

  Serial.println("Boat Control System Started");
}
unsigned long long last_time = millis();
unsigned long long last_sonar_t = millis();

void loop() {
    // Check if data is available from the Bluetooth module
  if (bluetooth.available()) {
    last_time = millis();
    char command = bluetooth.read(); // Read one character from Bluetooth
    Serial.print("Received: ");
    Serial.println(command);        // Print the character to Serial Monitor

    switch (command) {
      case 'F':
        moveForward();
        break;

      case 'B':
        moveBackward();
        break;

      case 'L':
        turnLeft();
        break;

      case 'R':
        turnRight();
        break;

      case 'S':
        stopMotors();
        break;

      case 'T':
        openDoor();
        break;

      case 'X':
        closeDoor();
        break;

      case 'P':
        pumpStart();
        break;

      default:
        Serial.println("Unknown command");
        break;
    }
  }
  if( millis() -last_time > 400) {
    stopMotors();
    stopPump();
  }

  if( millis() -last_sonar_t > 3000){
    last_sonar_t = millis();
    measure_distance();
//    tem_update();
  }

}

// Motor control functions
void moveForward() {
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  Serial.println("Moving forward");
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_FORWARD, 0);
  analogWrite(LEFT_MOTOR_BACKWARD, 255*RIGHT_FACT);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
  Serial.println("Moving backward");
}

void turnLeft() {
  digitalWrite(LEFT_MOTOR_FORWARD,HIGH );
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  Serial.println("Turning left");
}

void turnRight() {
  // digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD,LOW );
  Serial.println("Turning right");
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  Serial.println("Motors stopped");
}

// Servo control functions
void openDoor() {
  doorServo.write(DOOR_OPEN_ANGLE);
  Serial.println("Door opened");
}

void closeDoor() {
  doorServo.write(DOOR_CLOSED_ANGLE);
  Serial.println("Door closed");
}

void stopPump(void){
  digitalWrite(pump, LOW);
}

void pumpStart(void){
  digitalWrite(pump, HIGH);
}

void measure_distance(void){
  tone(SONAR, 1000, 200);
}
