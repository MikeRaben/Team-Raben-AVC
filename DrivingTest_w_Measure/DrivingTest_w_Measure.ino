#include <I2C.h>
#include "RunningMedian.h"
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.


int fw = 3;
int rv = 5;
int lf = 6;
int rt = 9;

long distTraveled;
int leg;

bool moving, frontClear;

void setup() {
  //Lidar Lite setup
  Serial.begin(9600); //Opens serial connection at 9600bps.     
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  
  //Setup motor and steering pins
  pinMode(fw, OUTPUT);
  pinMode(rv, OUTPUT);
  pinMode(lf, OUTPUT);
  pinMode(rt, OUTPUT);
  
  distTraveled = 0;
  leg = 1;
  
  forward();
  moving = true;
}

void loop() {
  // put your main code here, to run repeatedly:   
  if (measure() < 60){
    frontClear = false;
    stopAll();
    //code to scan L and R and decide on path
    Serial.println("Stopping");
  } else {
    frontClear = true;
    if(!moving){forward();}
    compass();
  // use distTraveled to determine which leg we're on
  // get compass measurement and adjust steering to match
  
  }
}

void forward(){
  //Drives forward for 1 second
  digitalWrite(rv, LOW);
  digitalWrite(fw, HIGH);
  moving = true;
}

void reverse(){
  //Drives forward for 1 second
  digitalWrite(fw, LOW);
  digitalWrite(rv, HIGH);
  delay(100);
  digitalWrite(rv, LOW);
  
}
void stopAll(){
  digitalWrite(fw, LOW);
  digitalWrite(rv, LOW);
  digitalWrite(lf, LOW);
  digitalWrite(rt, LOW);
  moving = false;
}

void leftTurn(){
  digitalWrite(lf, HIGH);
  delay(100);
  digitalWrite(lf, LOW);
}
void rightTurn(){
  digitalWrite(rt, HIGH);
  delay(100);
  digitalWrite(rt, LOW);
}

int measure(){
  RunningMedian samples = RunningMedian(15);
  samples.clear();

  //Prepare Lidar Lite
  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }
  for (int i = 0; i < 30; i++){
  
  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  samples.add(distance);
  }
  return samples.getMedian();
}



int compass(){
  //TODO: Add code to get heading from magnetometer
  int heading = 0;
  
  return heading;
}


void odometry(){
  //TODO: configure wheel encoder possibly use interrupt to increment count
}
