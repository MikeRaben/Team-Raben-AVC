//sweeping servo and taking lidar measurements
#include <I2C.h>
#include <RunningMedian.h>
#include <Servo.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

Servo myServo;

int servoL = 180;
int servoC = 90;
int servoR = 0;

int servoDelay = 100;
int servoPin = 10;

int stopDist = 75;

bool frontClear, leftClear, rightClear;

RunningMedian samples = RunningMedian(30);

void setup() {
  Serial.begin(9600); //Opens serial connection at 9600bps.
      
  Serial.println("Starting lidar");
  //Lidar Lite setup
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
  Serial.println("lidar success");

  myServo.attach(servoPin);
}

void loop() {
  lookAround();
  delay(500);
}

int measure(){
	samples.clear();

	//Prepare Lidar Lite
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

void lookAround(){
	lookLeft();
	leftClear = measure() < stopDist;
        Serial.println(leftClear);
	
	lookCenter();
	frontClear = measure() < stopDist;
        Serial.println(frontClear);
	
	lookRight();
	rightClear = measure() < stopDist;
        Serial.println(rightClear);
	
	lookCenter();
}

void lookLeft(){
	myServo.write(servoL);
	delay(servoDelay);
}

void lookRight(){
	myServo.write(servoR);
	delay(servoDelay);
}

void lookCenter(){
	myServo.write(servoC);
	delay(servoDelay);
}
