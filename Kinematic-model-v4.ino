#include <Pololu3piPlus32U4.h>
#define GOAL_DOTS_NUMBER 4

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;
ButtonC buttonC;
ButtonB buttonB;
LCD display;


const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";

bool rightDirection = false;
bool goalReached = false;

int V_MAX = 300;

//Square dots
int pathYCoordinates [GOAL_DOTS_NUMBER] = {300, -300, -300, 300};
int pathXCoordinates [GOAL_DOTS_NUMBER] = {-300, -300, 300, 300};

int currentGoalDot = 0;

int desiredPointX = -30;
int desiredPointY = 30;

float desiredAngle = 100;

float distance = 1000;

int prevLeftTick = 0;
int prevRightTick = 0;

float currentX = 0;
float currentY = 0;
float currentAngle = 0;

const int R = 16;
const int L = 80;

float DL_handler (){
  int16_t countsLeft = encoders.getCountsLeft();
  float dl = 2*3.14159*R*(countsLeft - prevLeftTick)/908;
  prevLeftTick = countsLeft;
  return dl;
}

float DR_handler(){
  int16_t countsRight = encoders.getCountsRight();
  float dr = 2*3.14159*R*(countsRight - prevRightTick)/906;
  prevRightTick = countsRight;
  return dr;
}

float DC_handler(){
  return (DR_handler() + DL_handler())/2;
}

void reachDistance(){
  float tickGoal = (distance*907)/(2*3.14159*R);
  int encoders_count = 0;
  while(encoders_count < tickGoal){
    encoders_count = (encoders.getCountsLeft()+encoders.getCountsRight())/2;
    calculateNewRobotCordinate();
    motors.setSpeeds(V_MAX, V_MAX);
  }
}

void distanceCheckHandler(){
  calculateDistanceToGoal();

  reachDistance();

  // Serial.println(distance);
  if(currentGoalDot < GOAL_DOTS_NUMBER){
    calculateGoalAngle(pathYCoordinates[currentGoalDot], pathXCoordinates[currentGoalDot]);
    currentGoalDot++;
    rightDirection=false;
  }else{
    //Uncomment next two lines for repeating of following dots
    calculateGoalAngle(pathYCoordinates[0], pathXCoordinates[0]);
    currentGoalDot = 1;

    //Comment line bellow for infinity dots follow
    // goalReached = true;
  }
}

void calculateNewRobotCordinate(){
  currentY = currentY + DC_handler()*sin(currentAngle);
  currentX = currentX + DC_handler()*cos(currentAngle);
}

float calculateCurrentAngle(){
  currentAngle = currentAngle + ((DR_handler() - DL_handler())/L);
  return currentAngle;
}

void calculateDistanceToGoal(){
  float y = pow((currentY - desiredPointY),2);
  float x = pow((currentX - desiredPointX),2);
  distance = sqrt(x+y);
}

void calculateGoalAngle (int newY, int newX ){
  desiredAngle = atan2(newY - currentY, newX - currentX);
  desiredPointY = newY;
  desiredPointX = newX;
  rightDirection=false;
}

// Update the screen with encoder counts
void displayEncoderCounts(int16_t countsLeft, int16_t countsRight){
    display.noAutoDisplay();
    display.clear();
    display.print(countsLeft);
    display.gotoXY(0, 1);
    display.print(countsRight);
    display.display();
}

void setup()
{
  calculateGoalAngle(pathYCoordinates[0], pathXCoordinates[0]);
  currentGoalDot++;
  displayEncoderCounts(0,0);
  delay(2000);
}

void loop()
{
  static uint8_t lastDisplayTime;
  static uint8_t displayErrorLeftCountdown = 0;
  static uint8_t displayErrorRightCountdown = 0;

  calculateCurrentAngle();


  if((currentAngle-0.01 < desiredAngle  && desiredAngle < currentAngle+0.01) || rightDirection){
    if(!rightDirection){
      prevLeftTick = 0;
      encoders.getCountsAndResetLeft();
      prevRightTick = 0;
      encoders.getCountsAndResetRight();
    }
    rightDirection = true;
    calculateNewRobotCordinate();
    distanceCheckHandler();

    if(goalReached){
      motors.setSpeeds(0, 0);
      delay(10000);
    }else{
      motors.setSpeeds(V_MAX, V_MAX);
    }
  }else if (currentAngle < desiredAngle){
    motors.setSpeeds(-V_MAX, V_MAX);
  }else if (currentAngle > desiredAngle){
    motors.setSpeeds(V_MAX, -V_MAX);
  }
  
  if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();

    int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();


    if (encoders.checkErrorLeft())
    { 
      // An error occurred on the left encoder channel.
      // Display it for the next 10 iterations and also beep.
      displayErrorLeftCountdown = 10;
      buzzer.playFromProgramSpace(encoderErrorLeft);
    }

    if (encoders.checkErrorRight())
    {
      // An error occurred on the left encoder channel.
      // Display for the next 10 iterations and also beep.
      displayErrorRightCountdown = 10;
      buzzer.playFromProgramSpace(encoderErrorRight);
    }

    //Display encoder counts
    displayEncoderCounts(countsLeft, countsRight);


    if (displayErrorLeftCountdown)
    {
      // Show an exclamation point on the first line to
      // indicate an error from the left encoder.
      display.gotoXY(7, 0);
      display.print('!');
      displayErrorLeftCountdown--;
    }
    if (displayErrorRightCountdown)
    {
      // Show an exclamation point on the second line to
      // indicate an error from the left encoder.
      display.gotoXY(7, 1);
      display.print('!');
      displayErrorRightCountdown--;
    }
  }

}
