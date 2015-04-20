
int fw = 3;
int rv = 5;
int lf = 6;
int rt = 9;

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
    blinkLED(1);
    leftTurn();
    delay(3000);
    blinkLED(2);
    rightTurn();
    delay(3000);
    blinkLED(3);
    forward(1000);
    delay(3000);
    blinkLED(4);
    reverse(1000);
    delay(3000);
}

void forward(int time){
  //Drives forward for 1 second
  analogWrite(fw, 255);
  delay(time);
  analogWrite(fw, 0);
}

void reverse(int time){
  //Drives forward for 1 second
  analogWrite(rv, 255);
  delay(time);
  analogWrite(rv, 0);
}
void stopAll(){
  analogWrite(fw, 0);
  analogWrite(rv, 0);
  analogWrite(lf, 0);
  analogWrite(rt, 0);
}

void leftTurn(){
  analogWrite(lf, 255);
  analogWrite(fw, 255);
  delay(1000);
  analogWrite(fw, 0);
  analogWrite(lf, 0);
}
void rightTurn(){
  analogWrite(rt, 255);
  analogWrite(fw, 255);
  delay(1000);
  analogWrite(fw, 0);
  analogWrite(rt, 0);
}
void blinkLED(int times){
  for (int x = 0; x < times; x++){
    analogWrite(13, 255);
    delay(200);
    analogWrite(13, 0);
    delay(200);
  }
}
