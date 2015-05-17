int fw = 3;
int rv = 5;
int lf = 6;
int rt = 9;

int turnDelay = 100; //update this to correct over/understeer

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
