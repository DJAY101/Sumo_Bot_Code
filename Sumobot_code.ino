#include <Ultrasonic.h>
#include <Adafruit_NeoPixel.h>

#define NEO_PIN 5
#define NUMPIXELS 24

// #define L_SPEED_PIN 9
// #define L_CONTROL_1 A2
// #define L_CONTROL_2 A1
#define L_SPEED_PIN 10
#define L_CONTROL_1 2
#define L_CONTROL_2 4

// #define R_SPEED_PIN 10
// #define R_CONTROL_1 A2
// #define R_CONTROL_2 A3
#define R_SPEED_PIN 11
#define R_CONTROL_1 9
#define R_CONTROL_2 8

#define IR_SENS_B A5
#define IR_SENS_F A4
#define ULTRA_ECHO 7
#define ULTRA_TRIG 6

class motorDriver {
public:
  motorDriver(int speedPin, int controlPin, int controlPin2) {
    m_speedPin = speedPin;
    m_controlPin = controlPin;
    m_controlPin2 = controlPin2;
  }
  void startDriver() {
    pinMode(m_speedPin, OUTPUT);
    pinMode(m_controlPin, OUTPUT);
    pinMode(m_controlPin, OUTPUT);

    digitalWrite(m_controlPin, HIGH);
    digitalWrite(m_controlPin2, LOW);
    m_direction = false;
  }

  void flipRotation() {
    m_direction = !m_direction;

    if (m_direction) {
      digitalWrite(m_controlPin, LOW);
      digitalWrite(m_controlPin2, HIGH);
    } else {
      digitalWrite(m_controlPin, HIGH);
      digitalWrite(m_controlPin2, LOW);
    }
  }


  void setSpeed(int speed) {
    analogWrite(m_speedPin, speed);
  }

  void setDirection(int value) {
    if (value >= 0) {
      m_direction = true;
    } else {
      m_direction = false;
    }

    if (m_direction) {
      digitalWrite(m_controlPin, LOW);
      digitalWrite(m_controlPin2, HIGH);
    } else {
      digitalWrite(m_controlPin, HIGH);
      digitalWrite(m_controlPin2, LOW);
    }
  }

  bool m_direction;
  int m_speedPin;
  int m_controlPin;
  int m_controlPin2;
};

class irSensor {
public:
  irSensor(int pin) {
    sensor_pin = pin;
  }
  startSens() {
    pinMode(sensor_pin, INPUT);
  }
  bool get() {
    return digitalRead(sensor_pin);
  }
  int sensor_pin;
};

class DriveBase {
public:
  DriveBase(motorDriver* left, motorDriver* right) {
    m_motorLeft = left;
    m_motorRight = right;
  }

  tankDrive(int left, int right) {
    m_motorLeft->setDirection(-left * flip_dir);
    m_motorLeft->setSpeed(abs(left));

    m_motorRight->setDirection(right * flip_dir);
    m_motorRight->setSpeed(abs(right));
  }


  int flip_dir = 1;
  motorDriver* m_motorLeft;
  motorDriver* m_motorRight;
};

motorDriver leftMotor{ L_SPEED_PIN, L_CONTROL_1, L_CONTROL_2 };
motorDriver rightMotor{ R_SPEED_PIN, R_CONTROL_1, R_CONTROL_2 };
DriveBase driveBase{ &leftMotor, &rightMotor };

Ultrasonic frontUltrasonic{ ULTRA_TRIG, ULTRA_ECHO };
irSensor frontSensor{ IR_SENS_F };
irSensor backSensor{ IR_SENS_B };

Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // put your setup code here, to run once:
  // init everything
  leftMotor.startDriver();
  rightMotor.startDriver();
  frontSensor.startSens();
  backSensor.startSens();
  pixels.begin(); //start the object
  Serial.begin(9600);

  driveBase.flip_dir = -1;
  delay(5000);
}

int counter = 0;


void showRed() {
  for (int i = (counter/8) % 2; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 255, 5, 0);
    pixels.setPixelColor(++i, 0, 0, 0);
  }

  if ((counter/8) % 2 == 1) {
    pixels.setPixelColor(0, 0, 0, 0);

  }

  pixels.show();
}

void showBlue() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 100, 100, 100);
  }
  pixels.show();
}

void showGreen() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, 0, 255, 0);
  }
  pixels.show();
}

int distance_front = 0;
int delta_time = 0;
int previous_time = 0;

int ultraSonic_timer = 300;

void loop() {
  // put your main code here, to run repeatedly:

  if(ultraSonic_timer > 250) {
   distance_front = frontUltrasonic.read();
   ultraSonic_timer = 0;
  }
  Serial.println(distance_front);

  // Serial.print("Ultrasonic Distance: ");
  // Serial.println(distance_front);
  // Serial.print(" IR_F: ");
  // Serial.print(frontSensor.get());
  // Serial.print(" IR_B: ");
  // Serial.println(backSensor.get());
  // delay(100);

  // leftMotor.setSpeed(255);
  // leftMotor.setDirection(1);
  // rightMotor.setSpeed(255);
  // rightMotor.setDirection(1);

  // if (frontSensor.get()) {
  //   showGreen();
  //   driveBase.tankDrive(-255, -255);
  //     delay(1000);
  //     driveBase.tankDrive(200, -200);
  //     delay (1000);
      // driveBase.tankDrive(255, 255);
    
  // }

  // driveBase.tankDrive(0, 100);

  if (distance_front < 45) {
    driveBase.tankDrive(250, 250);
  } else {
    driveBase.tankDrive(-140, 140);
  }


    // if (distance_front > 50) {
    //   showBlue();
    //   // if (counter < 200) {
    //   //   driveBase.tankDrive(-100, 100);
    //   // } else if (counter < 1000) {
    //   //   driveBase.tankDrive(100, -100);
    //   // } else {
    //   //   counter = 0;
    //   // }
    //   driveBase.tankDrive(-160, 160);
    // } else {
    //   // Serial.println("THIS");
    //   driveBase.tankDrive(250, 250);
    //   showRed();
    // }

  counter++;
  delta_time = millis() - previous_time;
  previous_time = millis();
  ultraSonic_timer += delta_time;
}
