#include <I2C.h>

#include <RunningMedian.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Wire.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.
#define encoderPin  2                        // Wheel encoder pin

int stopDist = 75;

int fw = 3;
int rv = 5;
int lf = 6;
int rt = 9;



int turnDelay = 100; //update this to correct over/understeer

bool moving;

Servo servo;

int servoL = 180;
int servoC = 90;
int servoR = 0;

int servoDelay = 175;
int servoPin = 10;

long distTraveled;
int legHeading [4] = {0, 90, 180, 270};
int legDist [5] = {100, 200, 300, 400};

bool frontClear, leftClear, rightClear;

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

RunningMedian samples = RunningMedian(30);
RunningMedian compSamples = RunningMedian(30);

void setup()
{
	Serial.begin(9600); //Opens serial connection at 9600bps.
      
	Serial.println("Starting lidar");
	//Lidar Lite setup
	I2c.begin(); // Opens & joins the irc bus as master
	delay(100); // Waits to make sure everything is powered up before sending or receiving data
	I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
	Serial.println("lidar success");
	/* Enable auto-gain */
	mag.enableAutoRange(true);
	Serial.println("Starting mag");
	/* Initialise the sensor */
	if(!mag.begin())
	{
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
		while(1);
	}
	Serial.println("mag success");

	//Setup servo on pin 10
	attachServo();

        //Setup wheel encoder
        pinMode(encoderPin, INPUT);
        attachInterrupt(0, encoderTick, CHANGE);
        
	//Setup motor and steering pins
	pinMode(fw, OUTPUT);
	pinMode(rv, OUTPUT);
	pinMode(lf, OUTPUT);
	pinMode(rt, OUTPUT);

	distTraveled = 50; // set to 0 before real use
	
	//while (digitalRead(4) == LOW) {}
}

void loop()
{
	lookAround();
	navigate();
	drive();
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

int compass(){
	compSamples.clear();

	for (int i = 0; i < 30; i++)
	{

		sensors_event_t event;
		mag.getEvent(&event);

		float Pi = 3.14159265359;

		// Calculate the angle of the vector y,x
		int heading = (int)(atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

		// Normalize to 0-360
		if (heading < 0)
		{
			heading = 360 + heading;
		}
		compSamples.add(heading);
	}
	return compSamples.getMedian();
}

void odometry(){
	//TODO: configure wheel encoder possibly use interrupt to increment count
}

void lookAround(){
	lookLeft();
	leftClear = measure() < stopDist;
	
	lookCenter();
	frontClear = measure() < stopDist;
	
	lookRight();
	rightClear = measure() < stopDist;
	
	lookCenter();
}

void navigate(){
	int targetHeading;
	int currentHeading = compass();
	
	if (distTraveled < legDist[0]){
		targetHeading = legHeading[0];
	} else if (distTraveled < legDist[1]){
		targetHeading = legHeading[1];
	} else if (distTraveled < legDist[2]){
		targetHeading = legHeading[2];
	} else if (distTraveled < legDist[3]){
		targetHeading = legHeading[3];
	}
	
	if (currentHeading + 5 < targetHeading){
		rightTurn();
	} else if (currentHeading - 5 > targetHeading){
		leftTurn();
	}
	
}

void drive(){
	if (frontClear){
		if(!moving){
			forward();
		}
	} else {
		//front not clear, check if left or right were clear
		if(rightClear){
			rightTurn();
		} else {
			if(leftClear){
				leftTurn();
			} else{ 
				//front left and right all blocked, back up
				stopAll();
				reverse();
			}
		}
	}
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
	if (moving){reverse();}
	
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

void attachServo(){
	servo.attach(servoPin);
}

void lookLeft(){
	servo.write(servoL);
	delay(servoDelay);
}

void lookRight(){
	servo.write(servoR);
	delay(servoDelay);
}

void lookCenter(){
	servo.write(servoC);
	delay(servoDelay);
}

void encoderTick(){
  distTraveled ++;
}
