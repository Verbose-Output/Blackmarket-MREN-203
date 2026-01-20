#include <Arduino.h>

int EA = 6; // Wheel PWM pin (must be a PWM capable pin)
int I1 = 7; // Wheel direction digital pin 1
int I2 = 4; // Wheel direction digital pin 2

int EB = 5; // Second wheel PWM pin (must be a PWM capable pin)
int I3 = 8; // Second wheel direction digital pin 1
int I4 = 2; // Second wheel direction digital pin 2

int delayTime = 2000; // Time to run each command in milliseconds
//int turnTime = 2000;  // Time to turn in milliseconds
//int pwmCommand = 200; // PWM command value [0-255]

void forward(int pwmCommand){

  analogWrite(EA, pwmCommand);
  analogWrite(EB, pwmCommand); 
  //left wheels
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);
  //right wheels
  digitalWrite(I3, HIGH);
  digitalWrite(I4, LOW);
}

void backward(int pwmCommand){

  analogWrite(EA, pwmCommand);
  analogWrite(EB, pwmCommand); 
  //left wheels
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  //right wheels
  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);
}

void turn(char direction, int pwmCommand, int duration){

  analogWrite(EA, pwmCommand);
  analogWrite(EB, pwmCommand); 

  if(direction == 'R'){
    //Left wheels
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    //Right wheels
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
  }
  else if(direction == 'L'){
    //Left wheels
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    //Right wheels
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  }

  delay(duration);

}

void stop(int duration){

  analogWrite(EA, 0);
  analogWrite(EB, 0); 
  //Left wheels
  digitalWrite(I1, LOW);
  digitalWrite(I2, LOW);
  //Right wheels
  digitalWrite(I3, LOW);
  digitalWrite(I4, LOW);

  delay(duration);

}

void circle(int leftSpeed, int rightSpeed, int duration){

  analogWrite(EA, leftSpeed);
  analogWrite(EB, rightSpeed);
  //left wheels
  digitalWrite(I1, LOW);
  digitalWrite(I2, HIGH);
  //right wheels
  digitalWrite(I3, LOW);
  digitalWrite(I4, HIGH);

  delay(duration);

}

void otherCircle(int leftSpeed, int rightSpeed, int duration){
  analogWrite(EA, leftSpeed);
  analogWrite(EB, rightSpeed);
  //left wheels
  digitalWrite(I1, HIGH);
  digitalWrite(I2, LOW);
  //right wheels
  digitalWrite(I3, HIGH);
  digitalWrite(I4, LOW);

  delay(duration);
}

void setup() {
  // Configure digital pins for output
  //left wheel
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  //right wheel
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
}


void loop() {
  //int u; // A variable for the motor PWM command [0-255]

  // Play with this code to write open loop commands to a wheel motor
  // forward(180);
  // delay(delayTime);
  // stop(delayTime);
  // turn('R', 220, 1750);
  // stop(delayTime);
  circle(230, 190, 4000); // Clockwise circle
  stop(delayTime);
  otherCircle(190, 230, 4000); // Clockwise circle
  stop(delayTime);
}


/*
We can make a function to give us a circular motion by making one wheel go
faster than the other. To do this accurately we would need to establish a relationship
between the PWM command values and the resulting wheel speeds. This can be done
experimentally by measuring the wheel speeds for different PWM commands...
*/