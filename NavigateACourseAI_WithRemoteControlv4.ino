/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h>
#include <IRremote.h>  

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
 //Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *frontLeft = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *rearLeft = AFMS.getMotor(2);
Adafruit_DCMotor *frontRight = AFMS.getMotor(3);
Adafruit_DCMotor *rearRight = AFMS.getMotor(4);
Servo servo1;
const int pingPin = 9;
const int xPin = 2;     // X output of the accelerometer
const int yPin = 1;     // Y output of the accelerometer
int pulseX, pulseY;

//remote controller config
int irReceiverPin = 3;
IRrecv irrecv(irReceiverPin); 
decode_results results;    

//RGB PINS
int redPin = 7;
int greenPin = 6;
int bluePin = 5;

// variables to contain the resulting accelerations
int accelerationX, accelerationY;
//right turn training
int turnRightTime = 20; //30;
double gammaRightTurn = .1;
long goalInchesFromWall = 1; //no more than 1 inch from the wall;


//left turn training
int turnLeftTime = 20; //32; //was 30
double gammaLeftTurn = .1;
long goalInchesFromWallLeftTurn = 1; //no more than 1 inch from the wall;
double leftTurnDistApartLow = 2.0; //1.4
double leftTurnDistApartHigh = 2.5; //1.7

//whichWayDidITurn
//this code tracks which direction the car turned
int whichWayDidITurn;

//tracks the current state of the agent - TRAININGMODE, QMODE
int TRAININGMODE = 12;
int QMODE = 13;
int mode = TRAININGMODE;

//long inches, cm;
const int NORTH = 0;
const int SOUTH = 1;
const int EAST = 2;
const int WEST = 3;
const int STOPPED = 4; //prior version 0
const int FORWARDM = 5;//prior version 1
const int LEFT = 6;
const int RIGHT = 7;

int state = STOPPED;

int motorSpeed = 50;
int motorSpeedRightTurn = 50; //was 75
int motorSpeedLeftTurn = 50;

int distanceToStop = 5.0; //30 inches - start stopping at this point


int const r = 4, c = 4;
int Q[r][c];
int R[r][c];
int frequencyTable[r][c];
double gammaQLearning = .9;
int currentDirection = NORTH;

//holds the distances of left (0), forward(1), right(2)
double distArray [3];
int distArrayLength = 3;
long randomNumber;
int littleBitSpeed = 45; //controls the speed for the remote control
int littleBitTime = 5;   //controls the time for the speed of the remote control
int goalStateDistance = 10; //car will go forward only if it measures a distance > this

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  Serial.println("create motor sheild!");  
  randomSeed(analogRead(0)); //seed the random number generator
  AFMS.begin();  // OR with a different frequency, say 1KHz
  Serial.println("frequency set!");  
  servo1.attach(10);
  servoFront();
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  Serial.println("setup complete");
  irrecv.enableIRIn();   
  //Serial.println("Training car to stop.");
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  servoFront();
  initR();
  initQ(); 
  delay(5000);  
  servoFront();
  flashLightReady(bluePin, 2); //setup complete
}


void loop() {
  int i;
  for(i = 0; i < 5; i++){
    zeroDistArray();
    flashLightReady(greenPin, 2);
    currentDirection = NORTH;
    QLearningAgent(currentDirection);  
    delay(250);
  }
  //User QLearning Table
  flashLightReady(redPin, 2);
  //switch the mode to QMODE
  mode = QMODE;
  while(true) {
    zeroDistArray();
    flashLightReady(greenPin, 2);
    currentDirection = NORTH;
    QLearningAgentSelectFromQ(currentDirection);
    delay(250);
  }
}

void flashLightReady(int pin, int count){
  int z;
  for(z = 0; z < count; z++){
    digitalWrite(pin, 255);
    delay(500);
    digitalWrite(pin, 0);
    delay(500);
  }
}

/****************************************************
  Right turn functions
*/
void turnRight(){
  uint16_t i;
  whichWayDidITurn = RIGHT;
  frontRight->run(BACKWARD);
  rearRight->run(BACKWARD);  
  frontLeft->run(FORWARD);
  rearLeft->run(FORWARD);
  
  //while(calcRotation() < 24){
  for (i=0; i< turnRightTime; i++) {
    frontLeft->setSpeed(motorSpeedRightTurn);  
    rearLeft->setSpeed(motorSpeedRightTurn);  
    frontRight->setSpeed(motorSpeedRightTurn);  
    rearRight->setSpeed(motorSpeedRightTurn);  
    delay(30);
  }
  frontLeft->run(RELEASE);
  rearLeft->run(RELEASE);
  frontRight->run(RELEASE);
  rearRight->run(RELEASE); 
  if(mode == TRAININGMODE){  
    adjust();
  }
}

void adjust(){
  Serial.println("adjust()");
  //listen to input from remote until exit button is pressed.
  while(true) {
  //Modify the turn   
    if (irrecv.decode(&results)) {
      Serial.println("********************");
      Serial.println(results.value);
      if(results.value == 534743) { //Right
          Serial.println("further to the right!");
          littleBitFurtherToRight();          
      } else if (results.value == 542903){ //Left
          Serial.println("further to the left!");
          littleBitFurtherToLeft();
      } else if (results.value == 524543) { //Break
          Serial.println("Exit adjust()");
          irrecv.resume(); // Receive the next value
          return;
      }
      delay(200);
      irrecv.resume(); // Receive the next value
    }
  }
}

/*
This accepts input from the remote and slightly turns the car to the right
*/
void littleBitFurtherToRight(){
  frontRight->run(BACKWARD);
  rearRight->run(BACKWARD);  
  frontLeft->run(FORWARD);
  rearLeft->run(FORWARD);
  int i;
  for (i=0; i < littleBitTime; i++) {
    frontLeft->setSpeed(littleBitSpeed);//motorSpeedRightTurn);  
    rearLeft->setSpeed(littleBitSpeed);  
    frontRight->setSpeed(littleBitSpeed);  
    rearRight->setSpeed(littleBitSpeed);  
    delay(30);
  }
  //update the left or right turn time accordingly
  switch(whichWayDidITurn){
    case LEFT:
      turnLeftTime -= 2;      
    case RIGHT:
      turnRightTime += 2;      
  } 
  
  frontLeft->setSpeed(0);//motorSpeedRightTurn);  
  rearLeft->setSpeed(0);  
  frontRight->setSpeed(0);  
  rearRight->setSpeed(0);  
  
  frontLeft->run(RELEASE);
  rearLeft->run(RELEASE);
  frontRight->run(RELEASE);
  rearRight->run(RELEASE); 
}

/*
This accepts input from the remote and turns the car slightly to the left
*/
void littleBitFurtherToLeft(){
  frontRight->run(FORWARD);
  rearRight->run(FORWARD);  
  frontLeft->run(BACKWARD);
  rearLeft->run(BACKWARD);
  int i;
  for (i=0; i < littleBitTime; i++) {
    frontLeft->setSpeed(littleBitSpeed);//motorSpeedRightTurn);  
    rearLeft->setSpeed(littleBitSpeed);  
    frontRight->setSpeed(littleBitSpeed);  
    rearRight->setSpeed(littleBitSpeed);  
    delay(30);
  }
  frontLeft->setSpeed(0);//motorSpeedRightTurn);  
  rearLeft->setSpeed(0);  
  frontRight->setSpeed(0);  
  rearRight->setSpeed(0);  
  
  //update the left or right turn time accordingly
  switch(whichWayDidITurn){
    case LEFT:
      turnLeftTime += 2;          
    case RIGHT:
      turnRightTime -= 2;      
  } 
  
  frontLeft->run(RELEASE);
  rearLeft->run(RELEASE);
  frontRight->run(RELEASE);
  rearRight->run(RELEASE); 
}

/****************************************************
  Left turn functions
*/
void turnLeft(){
  uint16_t i;
  whichWayDidITurn = LEFT;
  frontRight->run(FORWARD);
  rearRight->run(FORWARD);  
  frontLeft->run(BACKWARD);
  rearLeft->run(BACKWARD);
  
  //while(calcRotation() < 24){
  for (i=0; i< turnLeftTime; i++) {
    frontLeft->setSpeed(motorSpeedLeftTurn);  
    rearLeft->setSpeed(motorSpeedLeftTurn);  
    frontRight->setSpeed(motorSpeedLeftTurn);  
    rearRight->setSpeed(motorSpeedLeftTurn);  
    delay(30);
  }
  frontLeft->run(RELEASE);
  rearLeft->run(RELEASE);
  frontRight->run(RELEASE);
  rearRight->run(RELEASE); 
  
  if(mode == TRAININGMODE){  
    adjust();
  }
}

//forward
void moveCarForward(){
  if(canGoForward()){
      Serial.println("Forward!");
      goForward();
  } /*else if (mustStop()) {
      Serial.println("Stop!");
      stopCar();
  }*/
  
}

boolean canGoForward(){
  //servoFront();
  //delay(100);
  double dist = ping();
  delay(250);
  if(ping() >= distanceToStop && state == STOPPED){
    return true;
  } else {
    return false;
  }  
}

boolean mustStop(){
  double dist = ping();
  delay(100);
  if (dist < distanceToStop && state == FORWARDM){
    return true;
  } else {
     return false;
   }
}

void goForward(){
//  servoFront();
    state = FORWARDM;
    forward();
}

void forward(){
  uint8_t i;
  frontLeft->run(FORWARD);
  rearLeft->run(FORWARD);
  frontRight->run(FORWARD);
  rearRight->run(FORWARD);
  for (i=0; i<motorSpeed; i++) {
    /*if(!canGoForward()){
      stop();
      break;
    }*/
    frontLeft->setSpeed(i);  
    rearLeft->setSpeed(i);  
    frontRight->setSpeed(i);  
    rearRight->setSpeed(i);  
    delay(10);
  }  
  
  while(mustStop() == false){
    //keep checking when to stop
  }
  stopCar(); 
  
}

void stopCar(){
    state = STOPPED;
    stop();  
}

void stop(){
   uint8_t i;
   frontLeft->setSpeed(0);  
   rearLeft->setSpeed(0);  
   frontRight->setSpeed(0);  
   rearRight->setSpeed(0);  
  /*
  for (i=motorSpeed; i!=0; i--) {
   frontLeft->setSpeed(i);  
   rearLeft->setSpeed(i);  
   frontRight->setSpeed(i);  
   rearRight->setSpeed(i);  
   //delay(2);
  }*/
}

void backward(){
    uint8_t i;
  frontLeft->run(BACKWARD);
  frontRight->run(BACKWARD);
  rearLeft->run(BACKWARD);
  rearRight->run(BACKWARD);
  for (i=0; i<255; i++) {
    frontLeft->setSpeed(i);  
    rearLeft->setSpeed(i);  
    frontRight->setSpeed(i);  
    rearRight->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    frontLeft->setSpeed(i); 
    rearLeft->setSpeed(i);  
    frontRight->setSpeed(i);  
    rearRight->setSpeed(i);  
    delay(10);
  }
}

void servoRight(){
  servo1.write(0);
}
//SERVO
/*void servoRight(){
      uint8_t i;    
  for (i=255; i!=0; i--) {
    servo1.write(map(i, 0, 255, 0, 180)); 
    delay(3);
  }
}*/

void servoLeft(){
      uint8_t i;
      servo1.write(180);
  /*
  for (i=0; i<255; i++) {
     servo1.write(map(i, 0, 255, 0, 180)); 
     delay(3);
  }*/
}

void servoFront(){
  servo1.write(90);
}

//PING - DISTANCE SENSOR
double ping(){
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  double duration, inches; //, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  //cm = microsecondsToCentimeters(duration);
 
  Serial.print(inches);
  Serial.print("in, ");
  //Serial.print(cm);
  //Serial.print("cm");
  Serial.println();
 
//  delay(10);
  return inches;
}

double microsecondsToInches(double microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long calcRotation(){
 // read pulse from x- and y-axes:
  pulseX = pulseIn(xPin,HIGH);  
  pulseY = pulseIn(yPin,HIGH);
//  Serial.print(pulseX);
//    Serial.print("\t");
//   Serial.print(pulseY);
//     Serial.println();
  // convert the pulse width into acceleration
  // accelerationX and accelerationY are in milli-g's:
  // earth's gravity is 1000 milli-g's, or 1g.
  accelerationX = ((pulseX / 10) - 500) * 8;
  accelerationY = ((pulseY / 10) - 500) * 8;
  Serial.print("X acceleration: " );
    Serial.print(accelerationX );
  Serial.println();
  return accelerationX; 
}

/*
Initialize the Q model with 0s;
*/
void initQ(){
  uint8_t i, j;
  
  for(i=0; i < r; i++){
    for(j=0; j < c; j++){
      Q[i][j] = 0;
    }
  }
  for(i=0; i < r; i++){
    for(j=0; j < c; j++){
      frequencyTable[i][j] = 0;
    }
  }
}


/*
Initialize the Reward/transition matrix.
const int NORTH = 0;
const int SOUTH = 1;
const int EAST = 2;
const int WEST = 3;

-1 means the transition is not allowed.
e.g. North to South is not allowed; North to North is not allowed;
*/
void initR(){
    R[0][0] = 0; //N > N
    R[0][1] = -1; //N > S
    R[0][2] = 0;  //N > E
    R[0][3] = 0;  //N > W
    R[1][0] = -1; //S > N
    R[1][1] = -1; //S > S
    R[1][2] = 0;  //S > E
    R[1][3] = 0;  //S > W
    R[2][0] = 10; //E > N
    R[2][1] = -10;//E > S
    R[2][2] = -1; //E > E
    R[2][3] = -1; //E > W
    R[3][0] = 10; //W > N
    R[3][1] = -10;//W > S
    R[3][2] = -1; //W > E
    R[3][3] = -1; //W > W  
}

void zeroDistArray(){
  int i;
  for(i = 0; i < distArrayLength; i++){
    distArray[i] = -1;
  }
}


/*
returns an action given the current state
*/
int QLearningAgent( int currentState){
  int action = selectActionRandomly(currentState);
  while(!goalState()){
    Serial.print("in loop");
    Serial.println();    
    int actionConverted = convertAction(currentState, action);
    Serial.print("action selected: ");
    Serial.print(action);
    Serial.println();
    int newState = takeAction(currentState, actionConverted);
    currentState = newState;
    Serial.print("new state:" );
    Serial.print(newState);
    int reward = R[currentState][action];
    Serial.println();
    Serial.print("reward:");
    Serial.print(reward);
    int maxQ = getMaxQ(currentState);
    Serial.println();
    Serial.print("maxQ");
    Serial.print(maxQ);
    Q[currentState][action] = Q[currentState][action] + (reward + (gammaQLearning * maxQ));
    currentState = newState;
    action = selectActionRandomly(currentState);
    delay(1000);
  }
  delay(10000);   
}

/*
select from Q instead of random selection
*/
int QLearningAgentSelectFromQ( int currentState){
  int action = selectActionFromQ(currentState);
  while(!goalState()){
    Serial.print("in loop");
    Serial.println();    
    int actionConverted = convertAction(currentState, action);
    Serial.print("action selected: ");
    Serial.print(action);
    Serial.println();
    int newState = takeAction(currentState, actionConverted);
    currentState = newState;
    Serial.print("new state:" );
    Serial.print(newState);
    int reward = R[currentState][action];
    Serial.println();
    Serial.print("reward:");
    Serial.print(reward);
    int maxQ = getMaxQ(currentState);
    Serial.println();
    Serial.print("maxQ");
    Serial.print(maxQ);
    Q[currentState][action] = Q[currentState][action] + (reward + (gammaQLearning * maxQ));
    currentState = newState;
    action = selectActionFromQ(currentState);
    delay(1000);
  }
  delay(10000);   
}

/*
Exploration function
*/
double f(){
}

/*
Goal state is when the car cannot turn left, right or move forward.
*/
boolean goalState(){  
  Serial.println("goalState()");
  uint8_t i;
  if(distArray[0] == -1  && distArray[1] == -1 && distArray[2] == -1 ){
    Serial.println("return false");
    return false;
  }
  
  for(i = 0; i < distArrayLength; i++){
    if(distArray[i] > goalStateDistance){
      Serial.println("return false");
      return false;
    }
  } 
  servoFront();
  delay(1000);
  int z;
  for(z = 0; z < 5; z++){
    digitalWrite(greenPin, 255);
    delay(500);
    digitalWrite(greenPin, 0);
    delay(500);
  }
  Serial.println("return true");
  return true; 
}

int convertAction(int currentStatel, int action){

  if(currentStatel == NORTH){
    switch(action){
      case 0:
        return WEST;
      case 1:
        return NORTH;
      case 2: 
        return EAST;
    }
  } else if (currentStatel == SOUTH) {
    switch(action){
      case 0://left
        return EAST;
      case 1: //forward
        return SOUTH;
      case 2: //right
        return WEST;
    }
  } else if (currentStatel == EAST) {
    switch(action){
      case 0://left
        return NORTH;
      case 1: //forward
        return EAST;
      case 2: //right
        return SOUTH;
    }
  } else if (currentStatel == WEST){
     switch(action){
      case 0://left
        return SOUTH;
      case 1: //fforward
        return WEST;
      case 2: //right
        return NORTH;
    }
  }
}


/*
selects an action based on the precept.
*/
int selectAction(int currentState){
  double temp;
  int i;
  for(i = 0; i < distArrayLength; i++){
    distArray[i] = 0;
  }
  
  servoLeft();
  delay(1000);
  double distance = ping();
  delay(1000);
  //distArray[direction(currentState,LEFT)] = distance;
  distArray[0] = distance;
  Serial.println("distance left:");
  Serial.println(distance);
  
  servoFront();
  delay(1000);
  distance = ping();
  delay(1000);
  //distArray[currentState] = distance;
  distArray[1] = distance;
  Serial.println("distance front:");
  Serial.println(distance);  
    
  servoRight();
  delay(1000);
  distance = ping();
  delay(1000);
  //distArray[direction(currentState,RIGHT)] = distance;
  distArray[2] = distance;
  Serial.println("distance right:");
  Serial.println(distance);
  
  servoFront(); 
  delay(1000); 
  temp = max(distArray[0], distArray[1]);    
  for (i = 2; i < distArrayLength; i++){
    temp = max(temp, distArray[i]);    
  }
  for(i = 0; i < distArrayLength; i++){
    if(distArray[i] == temp){
      return i;
    }
  }
}

/*
selects an action based on the precept.
*/
int selectActionRandomly(int currentState){
  double temp;
  int i;
  for(i = 0; i < distArrayLength; i++){
    distArray[i] = 0;
  }
  
  servoLeft();
  delay(800);
  double distance = ping();
  delay(1000);
  //distArray[direction(currentState,LEFT)] = distance;
  distArray[0] = distance;
  Serial.println("distance left:");
  Serial.println(distance);
  
  servoFront();
  delay(800);
  distance = ping();
  delay(1000);
  //distArray[currentState] = distance;
  distArray[1] = distance;
  Serial.println("distance front:");
  Serial.println(distance);  
    
  servoRight();
  delay(800);
  distance = ping();
  delay(1000);
  //distArray[direction(currentState,RIGHT)] = distance;
  distArray[2] = distance;
  Serial.println("distance right:");
  Serial.println(distance);
  
  servoFront(); 
  delay(800); 
  
  //check how many actions can be taken from the current state
  int optionsAvailable = 0;
  for(i = 0; i < 3; i++){
    if(distArray[i] > goalStateDistance) {
      optionsAvailable = optionsAvailable + 1;
    }
  }  
  
  //if there is only one then return that action
  if( optionsAvailable == 1){
    for(i = 0; i < 3; i++){
      if(distArray[i] > goalStateDistance){ 
        return i;
      }
    }

  //if there is more than one then randomly choose an action
  } else if (optionsAvailable > 1) {
    return randomlySelectAction();          
  }
}

//randomly select the action
int randomlySelectAction(){
    Serial.print("");
    int selection = random(0,3);
    //keep choosing until you find one that has a distance greater than the goalStateDistance
    while( distArray[selection] <= goalStateDistance){ 
      selection = random(0,3);
    }
    return selection;
}

/*
selects an action based on the precept.
*/
int selectActionFromQ(int currentState){
  double temp;
  int i;
  for(i = 0; i < distArrayLength; i++){
    distArray[i] = 0;
  }
  
  servoLeft();
  delay(800);
  double distance = ping();
  delay(1000);
  //distArray[direction(currentState,LEFT)] = distance;
  distArray[0] = distance;
  Serial.println("distance left:");
  Serial.println(distance);
  
  servoFront();
  delay(800);
  distance = ping();
  delay(1000);
  //distArray[currentState] = distance;
  distArray[1] = distance;
  Serial.println("distance front:");
  Serial.println(distance);  
    
  servoRight();
  delay(800);
  distance = ping();
  delay(1000);
  //distArray[direction(currentState,RIGHT)] = distance;
  distArray[2] = distance;
  Serial.println("distance right:");
  Serial.println(distance);
  
  servoFront(); 
  delay(800); 
  
  //check how many actions can be taken from the current state
  int optionsAvailable = 0;
  int validActions[4]; //used to store valid actions
  int validActionsIndex = 0; //number of valid actions is this + 1
  for(i = 0; i < 3; i++){
    if(distArray[i] > goalStateDistance) {
      optionsAvailable = optionsAvailable + 1;
      validActions[validActionsIndex] = i;
      validActionsIndex = validActionsIndex + 1;
    }
  }  
  
  //if there is only one then return that action
  if( optionsAvailable == 1){
    for(i = 0; i < 3; i++){
      if(distArray[i] > goalStateDistance){ 
        return i;
      }
    }

//if there is more than one then choose max from Q
  } else if (optionsAvailable > 1) {
    int i;
    int validAction;
    int convertedAction;
    boolean areAllZero = true;
    //check if all the options have a Q = 0;
    for(i = 0; i < validActionsIndex + 1; i++){
      validAction = validActions[i]; //get the valid action (0, 1, or 2) from valid actions index
      convertedAction = convertAction(currentState, validAction);
      if(Q[currentState][convertedAction] != 0) {
        areAllZero = false;
        break;
      }
    }
    
    //if all are zero then randomly choose a value from Q or just select any avaliable action
    if(areAllZero){
      Serial.println("Randomly choose and action from Q");
      return randomlySelectAction();
    }
    
    //CONTINUE if they are all not zero then select the action based on the maximum Q value    
    int maxValue = -10;
    int maxSelection;
    //loop through the valid actions and select the one with the highest Q value
    for(i = 0; i < validActionsIndex + 1; i++){
      validAction = validActions[i]; //get the valid action (0, 1, or 2) from valid actions index
      convertedAction = convertAction(currentState, validAction);
      if(Q[currentState][convertedAction] > maxValue) {
        maxValue = Q[currentState][convertedAction];
        maxSelection = convertActionToLeftFrontRight(currentState, convertedAction);
      }
    }
    //in the instance that all the available actions are 0, then it will always select the last one
    return maxSelection;
  } else {
    Serial.println("selectActionFromQ() did not select an action this is a problem!.");
    //this mean that it is in the goal state
  }
}

int convertActionToLeftFrontRight(int currentStatel, int action){
  if(currentStatel == NORTH){
    switch(action){
      case NORTH:
        return 1; //forward
      case SOUTH:
        return -1; //NOT POSSIBLE
      case EAST: 
        return 2; //right
      case WEST:
        return 0; //LEFT
    }
  } else if (currentStatel == SOUTH){
      switch(action){
      case NORTH:
        return -1; //not possible
      case SOUTH:
        return 1; //forward
      case EAST: 
        return 0; //left
      case WEST:
        return 2; //right
      }
  } else if (currentStatel == EAST) {
    switch(action){
      case NORTH:
        return 0; //left
      case SOUTH:
        return 2; //right
      case EAST: 
        return 1; //forward
      case WEST:
        return -1; //not possible
    }
  } else if (currentStatel == WEST) {
      switch(action){
      case NORTH:
        return 2; //right
      case SOUTH:
        return 0; //left
      case EAST: 
        return -1; //not possible
      case WEST:
        return 1; //forward
  }
}
}

int takeAction(int currentState, int action){
  Serial.println();
  Serial.println("takeAction()");
  if(currentState == NORTH && action == NORTH) {
    Serial.println("CurrentState=NORTH;action=NORTH");
    moveCarForward();
 
    //delay(2000);
    return NORTH;
  } else if(currentState == NORTH && action == EAST){
    Serial.println("CurrentState=NORTH;action=EAST");
    turnRight();
    moveCarForward();
    return EAST;
  } else if(currentState == NORTH && action == WEST){
    Serial.println("CurrentState=NORTH;action=WEST");
    turnLeft();
    moveCarForward();
    return WEST;
  } else if(currentState == SOUTH && action == SOUTH){
    Serial.println("CurrentState=SOUTH;action=SOUTH");
    moveCarForward();
    return SOUTH;
  } else if(currentState == SOUTH && action == EAST){
    Serial.println("CurrentState=SOUTH;action=EAST");
    turnLeft();
    moveCarForward();
    return EAST;
  } else if(currentState == SOUTH && action == WEST){
    Serial.println("CurrentState=SOUTH;action=WEST");
    turnRight();
    moveCarForward();
    return WEST;
  } else if(currentState == EAST && action == EAST){
     Serial.println("CurrentState=EAST;action=EAST");

    moveCarForward();
    return EAST;
  } else if(currentState == EAST && action == NORTH){
     Serial.println("CurrentState=EAST;action=NORTH");
    turnLeft();
    moveCarForward();
    return NORTH;
  } else if(currentState == EAST && action == SOUTH){
     Serial.println("CurrentState=EAST;action=SOUTH");
    turnRight();
    moveCarForward();
    return SOUTH;
  } else if(currentState == WEST && action == WEST){
    Serial.println("CurrentState=WEST;action=WEST");
    moveCarForward();
    return WEST;
  } else if(currentState == WEST && action == NORTH){
     Serial.println("CurrentState=WEST;action=NORTH");
    turnRight();
    moveCarForward();
    return NORTH;
  } else if(currentState == WEST && action == SOUTH){
     Serial.println("CurrentState=WEST;action=SOUTH");
    turnLeft();
    moveCarForward();
    return SOUTH;
  }  
}

int getMaxQ(int newState){  
  if(newState == NORTH){
    return max(Q[newState][EAST], Q[newState][WEST]);
  } else if(newState == SOUTH){
    return max(Q[newState][EAST], Q[newState][WEST]);
  } else if(newState == EAST){
    return max(Q[newState][NORTH], Q[newState][SOUTH]);
  } else if(newState == WEST){
    return max(Q[newState][NORTH], Q[newState][SOUTH]);
  }
}

int direction(int currentState, int servoDirection){
  if(currentState == NORTH && servoDirection == LEFT){
    return WEST;
  }else if (currentState == NORTH && servoDirection == RIGHT){
    return EAST;
  }else if (currentState == SOUTH && servoDirection == LEFT){
    return EAST;
  }else if (currentState == SOUTH && servoDirection == RIGHT){
    return WEST;
  }else if (currentState == EAST && servoDirection == LEFT){
    return NORTH;
  }else if (currentState == EAST && servoDirection == RIGHT) {
    return SOUTH;
  }else if (currentState == WEST && servoDirection == LEFT){
    return SOUTH;
  }else if (currentState == WEST && servoDirection == RIGHT) {
    return NORTH;
  }
}
