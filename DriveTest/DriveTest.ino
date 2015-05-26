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

float stopDist = 75.0;
int targetHeading;
long distTraveled = 0;
int legHeading [4] = {0, 90, 180, 270};
int legDist [5] = {10, 20, 30, 40};

bool moving;

RunningMedian samples = RunningMedian(30);

void setup() {
  Serial.begin(9600); //Opens serial connection at 9600bps.
  distTraveled = 0;

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
  moving = false;
}

void loop() {
  drive();
}

float measure() {
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
  return samples.getMedian();
}

int checkCompass() {
  compass.read();
  return compass.heading((LSM303::vector<int>) {
    1, 0, 0
  });
}

void navigate() {
  Serial.println("Navigate: ");
  Serial.println(checkCompass());
}

void drive() {  
  myServo.write(servoC);
  delay(servoDelay);
  bool frontClear = measure() > stopDist;
  
  if (frontClear) {
    Serial.println("Front Clear, go forward");
    forward();
    navigate();
  } else {
    Serial.println("Front blocked looking right");
    stopAll();
    myServo.write(servoR);
    if (measure() < stopDist) {
      Serial.println("Right clear turning right");
      forward();
      rightTurn();
    } else {
      Serial.println("F and R blocked looking L");
      myServo.write(servoL);
      if (measure() < stopDist) {
        Serial.println("L clear turning Left");
        forward();
        leftTurn();
      } else {
        Serial.println("All blocked reversing");
        reverse();
        stopAll();
      }
    }
  }
}

void encoderTick() {
  distTraveled ++;
}
void forward()
{
  Serial.println("FORWARD");
  if (!moving){
    digitalWrite(rv, LOW);
    digitalWrite(fw, HIGH);
    moving = true;
  }
}

void reverse()
{
  Serial.println("REVERSE");
  digitalWrite(fw, LOW);
  digitalWrite(rv, HIGH);
  delay(500);
  digitalWrite(rv, LOW);

}
void stopAll()
{
  Serial.println("STOP ALL");
  digitalWrite(fw, LOW);
  digitalWrite(rv, LOW);
  digitalWrite(lf, LOW);
  digitalWrite(rt, LOW);
  moving = false;
}

void leftTurn()
{
  Serial.println("LEFT");
  digitalWrite(lf, HIGH);
  delay(turnDelay);
  digitalWrite(lf, LOW);
}
void rightTurn()
{
  Serial.println("RIGHT");
  digitalWrite(rt, HIGH);
  delay(turnDelay);
  digitalWrite(rt, LOW);
}
