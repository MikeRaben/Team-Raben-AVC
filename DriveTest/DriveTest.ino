#include <I2C.h>
#include <RunningMedian.h>
#include <Servo.h>
#include <LSM303.h>
#include <Wire.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

#define encoderPin  2                        // Wheel encoder pin

LSM303 compass;

Servo myServo;

int servoL = 180;
int servoC = 90;
int servoR = 0;
int servoDelay = 300;
int servoPin = 10;

int fw = 3;
int rv = 5;
int lf = 6;
int rt = 9;

int turnDelay = 300; //update this to correct over/understeer

int stopDist = 75;
int targetHeading;
long distTraveled = 0;
int legHeading [4] = {0, 90, 180, 270};
int legDist [5] = {10, 20, 30, 40};

bool frontClear, leftClear, rightClear;
bool moving;

RunningMedian samples = RunningMedian(30);

void setup() {
  Serial.begin(9600); //Opens serial connection at 9600bps.
  distTraveled = 0;
  frontClear = true;

  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);

  //compass setup
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -42, -700, -432
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +971, +238, +464
  };
  //Lidar Lite setup
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  myServo.attach(servoPin);
  myServo.write(servoC);

  //Setup wheel encoder
  //  pinMode(encoderPin, INPUT);
  //  attachInterrupt(0, encoderTick, HIGH);
}

void loop() {
  drive();
}

int measure() {
  samples.clear();
  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses
  while (nackack != 0)  // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
  {
    nackack = I2c.write(LIDARLite_ADDRESS, RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }
  for (int i = 0; i < 30; i++)
  {

    byte distanceArray[2]; // array to store distance bytes from read function

    // Read 2byte distance from register 0x8f
    nackack = 100; // Setup variable to hold ACK/NACK resopnses

    // Read 2byte distance from register 0x8f
    nackack = 100; // Setup variable to hold ACK/NACK resopnses
    while (nackack != 0)  // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    {
      nackack = I2c.read(LIDARLite_ADDRESS, RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
      delay(1); // Wait 1 ms to prevent overpolling
    }
    int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
    samples.add(distance);
  }
  return (int) samples.getMedian();
}

int checkCompass() {
  compass.read();
  return compass.heading((LSM303::vector<int>) {
    0, 1, 0
  }); //? switch 1s around to get X heading
}

void lookAround() {
  myServo.write(servoL);
  delay(servoDelay);
  leftClear = measure() < stopDist;

  myServo.write(servoC);
  delay(servoDelay);
  frontClear = measure() < stopDist;

  myServo.write(servoR);
  delay(servoDelay);
  rightClear = measure() < stopDist;

  myServo.write(servoC);
  delay(servoDelay);
}

void navigate() {
  Serial.println("Navigate: ");
  Serial.print(checkCompass());
}

void drive() {
  myServo.write(servoC);
  delay(servoDelay);
  frontClear = measure() < stopDist;

  if (frontClear && !moving) {
    forward();
  }
  if (frontClear && moving) {
    navigate();
  }
  if (!frontClear) {
    stopAll();
    lookAround();
    if (rightClear) {
      forward();
      rightTurn();
    }
    if (leftClear) {
      forward();
      leftTurn();
    }
  }
  if (!frontClear && !rightClear && !leftClear) {
    stopAll();
    reverse();
    delay(500);
    stopAll();
  }
}

void encoderTick() {
  distTraveled ++;
}
void forward()
{
  digitalWrite(rv, LOW);
  digitalWrite(fw, HIGH);
  moving = true;
}

void reverse()
{
  digitalWrite(fw, LOW);
  digitalWrite(rv, HIGH);
  delay(100);
  digitalWrite(rv, LOW);

}
void stopAll()
{
  digitalWrite(fw, LOW);
  digitalWrite(rv, LOW);
  digitalWrite(lf, LOW);
  digitalWrite(rt, LOW);
  moving = false;
}

void leftTurn()
{
  digitalWrite(lf, HIGH);
  delay(turnDelay);
  digitalWrite(lf, LOW);
}
void rightTurn()
{
  digitalWrite(rt, HIGH);
  delay(turnDelay);
  digitalWrite(rt, LOW);
}
