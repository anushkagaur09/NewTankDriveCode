/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// rightFront           motor         1               
// leftFront            motor         3               
// leftBack             motor         7               
// rightBack            motor         12              
// roller               motor         8               
// Expansion            digital_in    A               
// shooterIndexer       digital_in    B               
// Intake               motor         9               
// Intake2              motor         10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//PID settings
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

int error; //SensorValue - DesiredValue --- positional value
int prevError = 0; //position 20 milliseconds ago
int derivative; //difference between error and previous error --- calculates speed
int totalError = 0;// totalError = totalError + error --- integral converts position to absement

//turn variables
int turnError; //SensorValue - DesiredValue --- positional value
int turnPrevError = 0; //position 20 milliseconds ago
int turnDerivative; //difference between error and previous error --- calculates speed
int turnTotalError = 0;

int desiredValue = 200;
int desiredTurnValue = 0;

bool enabledrivePID = true;
//switch to reset the Drive
bool resetDriveSensors = false;


int drivePID(){
  
  while(enabledrivePID){

    if(resetDriveSensors){
      resetDriveSensors = false;

      rightFront.setPosition(0, degrees);
      leftFront.setPosition(0, degrees);
      leftBack.setPosition(0, degrees);
      rightBack.setPosition(0, degrees);
    }
    //get the position of the motors
    int rightFrontPosition = rightFront.position(degrees);
    int leftFrontPosition = leftFront.position(degrees);
    int leftBackPosition = leftBack.position(degrees);
    int rightBackPosition = rightBack.position(degrees);
    //////////////////////////////////////////////////////////////////////////////////
    //lateral movement PID
    ////////////////////////////////////////////////////////////////////////
    //get the average of the four motors
    int averagePosition = (rightFrontPosition + leftFrontPosition + leftBackPosition + rightBackPosition)/4;

    error = averagePosition - desiredValue;

    derivative = error - prevError;

    //absement = position * time -- this is the integral
    totalError += error;

    //add everything up to a mootor power
    double lateralMotorPower = (error * kP + derivative * kD + totalError * kI)/12;

    //////////////////////////////////////////////////////////////////////////////////
    //turning PID
    int turnDifference = (rightFrontPosition - leftFrontPosition);

    turnError = turnDifference - desiredTurnValue;

    turnDerivative = turnError - turnPrevError;

    //absement = position * time -- this is the integral
    turnTotalError += turnError;

    //add everything up to a mootor power
    double turnMotorPower = (turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI)/12.0;
    ////////////////////////////////////////////////////////////////////////
    //putting in the motor power into the motor statements
    rightFront.spin(forward, lateralMotorPower + turnMotorPower , percent);
    leftFront.spin(forward, lateralMotorPower - turnMotorPower, percent);
    leftBack.spin(forward, lateralMotorPower - turnMotorPower, percent);
    rightBack.spin(forward, lateralMotorPower + turnMotorPower, percent);
    

    prevError = error;
    turnPrevError = turnError;

    vex::task::sleep(20);
  }

  return 1;

}

  
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  vex::task DriveCode(drivePID);
  //example of how to do PID in auton
  resetDriveSensors = true;
  desiredValue = 300; //move forward 300
  desiredTurnValue = 600; //turn 600

  vex::task::sleep(1000); // have it stop for a second

  desiredValue = 300; //move forward 300
  desiredTurnValue = 300; //turn 300

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
   

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
