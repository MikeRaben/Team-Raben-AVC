#include <I2C.h>
#include <RunningMedian.h>
#include <Servo.h>
#include <LSM303.h>
#include <Wire.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////
bool debugging = false;                     //prints debug messages for testing
bool objAvoid = false;                      //Use the lidar for avoiding, false for testing
long interval = 100;                        //Deley for debouncing reed switch
long navDelay = 750;                        //Avoid constantly making steering adjustments
int driveSpeed = 255;                       //between 0 and 255 below 90 doesnt move
int turnSpeed = 255;                        //constant for writing steering motor
float wiggle = 4.0;                         //+/- degrees steering can be off without adjusting
int turnDelay = 500;                        //used for object avoiding turns only
int stopDist = 75;                          //distance in cm to start avoiding objects
float degPerMilli = 0.087;                  //number of degrees turning for 1 milli will change heading

int legDist [5] = {41, 92, 133, 184, 225};     //Basketball Court
// int legDist[5] = {59, 338, 456, 735, 794};  //AVC Full Course
// int legDist[5] = {59, 185, 303, 429, 488};  //AVC Short Cut

/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

#define encoderPin  2   // Wheel encoder pin

volatile unsigned long lastTick, lastNav;

LSM303 compass;
Servo myServo;

int servoL = 135;
int servoC = 90;
int servoR = 45;
int servoDelay = 100;
int servoPin = 10;

int fw = 3;
int rv = 5;
int lf = 6;
int rt = 9;

int targetHeading;
long distTraveled;
int legHeading [4];
bool moving, frontClear, rightClear, leftClear, tracking;

RunningMedian samples = RunningMedian(50);      //Arrays to store measurements then return the median
RunningMedian headSamples = RunningMedian(50);  //Helps reduce sensor noise

void setup() {
  Serial.begin(9600);
  distTraveled = 0;

  //compass setup
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -257, -915, -717
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +961, +244, +372
  };

  //Calculate the heading for each leg
  legHeading[0] = checkCompass();
  for (int i = 1; i < 4; i++) {
    if (legHeading[i - 1] + 90 > 180) {
      legHeading[i] = legHeading[i - 1] - 270;
    } else {
      legHeading[i] = legHeading[i - 1] + 90;
    }
  }
  if (debugging) {
    Serial.println("Leg Headings");
    Serial.print(legHeading[0]);
    Serial.print("\t");
    Serial.print(legHeading[1]);
    Serial.print("\t");
    Serial.print(legHeading[2]);
    Serial.print("\t");
    Serial.println(legHeading[3]);
  }

  if (objAvoid) {
    //Lidar Lite setup
    I2c.begin(); // Opens & joins the irc bus as master
    delay(100); // Waits to make sure everything is powered up before sending or receiving data
    I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

    myServo.attach(servoPin);
    myServo.write(servoC);
  }
  //Setup wheel encoder
  pinMode(encoderPin, INPUT);
  attachInterrupt(0, encoderTick, RISING);
  moving = false;
  frontClear = true;
  tracking = true;
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
  for (int i = 0; i < 50; i++)
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
  headSamples.clear();
  for (int i = 0; i < 50; i++)
  {
    compass.read();
    int newHeading = compass.heading((LSM303::vector<int>) {
      1, 0, 0
    });
    if (newHeading > 180) {
      headSamples.add(newHeading - 360);
    } else {
      headSamples.add(newHeading);
    }
    delay(1);
  }
  return (int) headSamples.getMedian();
}

void navigate() {
  int newHeading;
  bool allowBigTurn;
  if (debugging) {
    Serial.println("Navigate");
    Serial.println(distTraveled);
  }
  long currentMillis = millis();
  if ((currentMillis - lastNav) > navDelay) {
    if (distTraveled < legDist[0]) {
      newHeading = legHeading[0];
    }
    if (distTraveled > legDist[0] && distTraveled < legDist[1]) {
      newHeading = legHeading[1];
    }
    if (distTraveled > legDist[1] && distTraveled < legDist[2]) {
      newHeading = legHeading[2];
    }
    if (distTraveled > legDist[2] && distTraveled < legDist[3]) {
      newHeading = legHeading[3];
    }
    if (distTraveled > legDist[3] && distTraveled < legDist[4]) {
      newHeading = legHeading[0];
    }
    if (distTraveled > legDist[4]) {
      stopAll();
      delay(5000);
    }
    if (newHeading != targetHeading) {
      allowBigTurn = true;
      targetHeading = newHeading;
    } else {
      allowBigTurn = false;
      targetHeading = newHeading;
    }

    int currentHeading = checkCompass();
    if (debugging) {
      Serial.println("Current Heading\t Target");
      Serial.print(currentHeading);
      Serial.print("\t");
      Serial.println(targetHeading);
    }
    int turnDeg = targetHeading - currentHeading;
    if (turnDeg < -180) {
      turnDeg += 360;
    }
    if (turnDeg > 180) {
      turnDeg -= 360;
    }
    lastNav = currentMillis;
    headingHold(turnDeg, allowBigTurn);
  }
}

void sweep() {
  myServo.write(servoC);
  delay(servoDelay);
  frontClear = measure() > stopDist;

  myServo.write(servoR);
  delay(servoDelay);
  rightClear = measure() > stopDist;

  myServo.write(servoL);
  delay(servoDelay);
  leftClear = measure() > stopDist;

  myServo.write(servoC);
  delay(servoDelay);
}

void drive() {
  if (objAvoid) {
    myServo.write(servoC);
    delay(servoDelay);
    frontClear = measure() > stopDist;
  }

  if (frontClear) {
    forward();
    navigate();
  } else {
    stopAll();
    reverse();
    sweep();
    if (rightClear) {
      forward();
      rightTurn();
    } else {
      if (leftClear) {
        forward();
        leftTurn();
      } else {
        reverse();
        stopAll();
      }
    }
  }
}

void encoderTick() {
  if (tracking) {
    long currentMillis = millis();
    if ((currentMillis - lastTick) > interval) {
      distTraveled ++;
      lastTick = currentMillis;
    }
  }
}
void forward()
{
  if (!moving) {
    analogWrite(rv, 0);
    analogWrite(fw, driveSpeed);
    moving = true;
  }
}

void reverse()
{
  tracking = false;
  analogWrite(fw, 0);
  analogWrite(rv, 255);
  delay(500);
  analogWrite(rv, 0);
  tracking = true;
}
void stopAll()
{
  analogWrite(fw, 0);
  analogWrite(rv, 0);
  analogWrite(lf, 0);
  analogWrite(rt, 0);
  moving = false;
}

void leftTurn()
{
  analogWrite(lf, turnSpeed);
  delay(turnDelay);
  analogWrite(lf, 0);
}
void rightTurn()
{
  analogWrite(rt, turnSpeed);
  delay(turnDelay);
  analogWrite(rt, 0);
}

void headingHold(float delta, bool bigTurn) {
  if (debugging) {
    Serial.println("Delta \t Big turn");
    Serial.print(delta);
    Serial.print("\t");
    Serial.println(bigTurn);
  }

  // minimize big swinging turns
  if (!bigTurn && delta > 30) {
    delta = delta / 3;
  }

  int thisDelay = (int) delta / degPerMilli;
  if (delta < -wiggle) {
    if (debugging) {
      Serial.println("Left to ");
      Serial.println(thisDelay);
    }
    analogWrite(lf, turnSpeed);
    delay(abs(thisDelay));
    analogWrite(rt, 0);
  }
  if (delta > wiggle) {
    if (debugging) {
      Serial.println("Right to ");
      Serial.println(thisDelay);
    }
    analogWrite(rt, turnSpeed);
    delay(abs(thisDelay));
    analogWrite(lf, 0);
  }
  analogWrite(lf, 0);
  analogWrite(rt, 0);
}
