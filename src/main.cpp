/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       acire                                                     */
/*    Created:      1/16/2026, 12:30:01 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "robotconfig.h"

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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  robotBrain.Screen.setCursor(1,1);
  robotBrain.Screen.print("DANGER  DANGER DANGER: Do NOT move the robot!!!!!");
  rightGpsSensor.calibrate();
  leftGpsSensor.calibrate();
  inertialSensor.calibrate();
  //vex::analog_in leftSideFront;
  //vex::analog_in leftSideBack;
  //vex::analog_in rightSideFront;
  //vex::analog_in rightSideBack;
  inertialSensor.resetHeading();
  leftMotors.setStopping(hold);
  rightMotors.setStopping(hold);
  

  wait(5000,msec);

  robotBrain.Screen.clearScreen();
  double lastHeading = 0.0;
  double currentHeading = 0.0;

  while (1) {
    robotBrain.Screen.setPenColor(color::white);  
    robotBrain.Screen.setCursor(1,1);
    robotBrain.Screen.print("LEFT  (%8.2f, %8.2f) heading:%5.2f", 
    leftGpsSensor.xPosition(), leftGpsSensor.yPosition(), leftGpsSensor.heading());

    robotBrain.Screen.setPenColor(color::white);  
    robotBrain.Screen.setCursor(2,1);
    robotBrain.Screen.print("RIGHT (%8.2f, %8.2f) heading:%5.2f", 
    rightGpsSensor.xPosition(), rightGpsSensor.yPosition(), rightGpsSensor.heading());
    
    robotBrain.Screen.setPenColor(color::white);  
    robotBrain.Screen.setCursor(3,1);
    robotBrain.Screen.print("Inertial heading:%5.2f", inertialSensor.heading());
    
    
    if (fabs(lastHeading - currentHeading) > .1) {
      robotBrain.Screen.setPenColor(color::yellow);
      robotBrain.Screen.setCursor(5,1);
      robotBrain.Screen.print("WARNING: Check Inertial Sensor");  
      robotBrain.Screen.setPenColor(color::white);  
    }

    if (leftGpsSensor.yPosition() == 0 || leftGpsSensor.xPosition() == 0 ) {
      robotBrain.Screen.setPenColor(color::yellow);
      robotBrain.Screen.setCursor(6,1);
      robotBrain.Screen.print("WARNING: Check left GPS"); 
      robotBrain.Screen.setPenColor(color::white);  
    }

    if (rightGpsSensor.yPosition() == 0 || rightGpsSensor.xPosition() == 0 ) {
      robotBrain.Screen.setPenColor(color::yellow);
      robotBrain.Screen.setCursor(7,1);
      robotBrain.Screen.print("WARNING: Check right GPS");    
      robotBrain.Screen.setPenColor(color::white);  
    }

  if(robotBrain.Battery.capacity()<50) {
      robotBrain.Screen.setCursor(11,1);
      robotBrain.Screen.setPenColor(color::yellow);
      robotBrain.Screen.print("LOW Battery. WARNING!!! Below 50% %d", robotBrain.Battery.capacity());
      robotBrain.Screen.setPenColor(color::white);  
  }
  
  lastHeading = currentHeading;
  currentHeading = inertialSensor.heading();

  wait(1000, msec);
  robotBrain.Screen.clearScreen();
  
  }
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

   //All my variables
  double currentHeading;
  double robotTurn;

  double turnKp = 0.50; //0.35
  double turnKd = 0.03; //0.02
  double turnKi = 0.0;
  double straightKp = 0.05;  //0.10
  double straightKd = 0.03;  //0.02

  double strafeKp = 0.50;

  double currentY;
  double currentX;
     
  //////////////////////////////////////////////////////////////////////////////////////////
  //                                        SKILLS ROUTINE!!
  //////////////////////////////////////////////////////////////////////////////////////////
  

    
  descore.set(true);  
  blockingWizard.set(true);
  driveLeftWithH(80, 1350);
  wait(100, msec);
  strafeWithDistanceL(455, strafeKp);
  wait(100, msec);
  AlignLeftSide();
  wait(100, msec);
  strafeWithDistanceL(455, strafeKp);
  wait(100, msec);
  unknownHammerDown();
  wait(100, msec);
  driveFoward(40, 550);      
  ramAndIntakeSkills(5);     

  ///////////////////////////////////////
  //Aligns with the long goal & scores
  
  wait(100, msec);
  driveReverse(100, 300);
  wait(100, msec);
  strafeWithDistanceL(440, strafeKp);
  wait(100, msec);
  AlignLeftSide();
  wait(100, msec);
  driveReverse(50, 700);
  wait(100, msec);
  strafeWithDistanceL(440, strafeKp);
  wait(100, msec);
  AlignLeftSide();
  blockingWizard.set(false);
  driveReverse(30, 600);
  scoringTopSkills(100, 3000);

  
  unknownHammerUp();
  wait(100, msec);
  driveFoward(50, 550);
  driveLeftWithH(-80, 900);

  currentHeading = leftGpsSensor.heading();
  robotTurn = 270 - currentHeading;
  rotateInertialClockWisePID(robotTurn, turnKp, turnKd, turnKi);

  currentX = leftGpsSensor.xPosition();
  // velocity of following was 100
  maintainConstantX_DirectionPositiveYLeft(straightKp, straightKd, currentX, 1000, 50);
  
  //currentHeading = leftGpsSensor.heading();
  robotTurn = 90; // currentHeading - 180;
  rotateInertialCounterClockWisePID(robotTurn, turnKp, turnKd, turnKi);

  ///////////////////////////////////////////////
  ////////CROSSES THE FIELD/////////////////////
  driveFoward(50, 300);
  strafeWithDistanceR(445, strafeKp);
  wait(100, msec);
  AlignRightSide();
  wait(100, msec);
  strafeWithDistanceR(445, strafeKp); 
  wait(100, msec);
  unknownHammerDown();
  wait(300, msec);
  driveFoward(50, 500);  //was 1500
  wait(100, msec);
  blockingWizard.set(true);
  wait(100, msec);
  ramAndIntakeSkills(5);      

  wait(100, msec);
  driveReverse(100, 300);
  wait(100, msec);
  strafeWithDistanceR(470, strafeKp);
  wait(100, msec);
  AlignRightSide();
  driveReverse(50, 700);
  wait(100, msec);
  strafeWithDistanceR(470, strafeKp);
  wait(100, msec);
  AlignRightSide();
  blockingWizard.set(false);
  driveReverse(30, 600);
  scoringTopSkills(100, 3000);
  unknownHammerUp();

  driveFoward(50, 400);
  rotateInertialCounterClockWisePID(90, turnKp, turnKd, turnKi);
  driveFoward(30, 500);
  currentX = rightGpsSensor.xPosition();
  maintainConstantX_DirectionNegativeYLeft(straightKp, straightKd, currentX, 800, 80);



  driveLeftWithH(-100, 1500);
  driveReverse(80, 300);
  driveFoward(80, 2700);    // was 1000

  for (int count = 0; count < 3; count++) {
        rightMotors.spin(fwd, 10, pct);
        leftMotors.spin(reverse, 10, pct);
        wait(100, msec);
        rightMotors.spin(reverse, 10, pct);
        leftMotors.spin(fwd, 10, pct);
        wait(100, msec);
        rightMotors.stop(hold);
        leftMotors.stop(hold);
    }

  frontIntake.spin(reverse, 100, pct);
  wait(500, msec);
  frontIntake.stop(coast);
  
  
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
  bool isRaised = false;  // Keeps track of whether the intake is up or down
  //unknownHammer.set(true);

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    
    leftMotors.spin(vex::directionType::fwd,  (controller1.Axis3.value() + (controller1.Axis1.value())), vex::velocityUnits::pct);
    rightMotors.spin(vex::directionType::fwd,  (controller1.Axis3.value() - (controller1.Axis1.value())), vex::velocityUnits::pct);
    leftMotors.setStopping(coast);
    rightMotors.setStopping(coast);
    rightStrafeMotor.spin(fwd, controller1.Axis4.value(), pct);
    leftStrafeMotor.spin(reverse, controller1.Axis4.value(), pct);

    if (controller1.Axis4.value() == 0 ){
      leftStrafeMotor.stop(hold);
      rightStrafeMotor.stop(hold);
    }

  
    if (controller1.ButtonA.pressing()){
      unknownHammer.set(false);
    } else if (controller1.ButtonX.pressing()){
      unknownHammer.set(true);
    }

    /*
    if (controller1.ButtonY.pressing()){
      descore.set(false);
    } else if (controller1.ButtonB.pressing()){
      descore.set(true);
    }
    */

    if (controller1.ButtonUp.pressing()){
      blockingWizard.set(false);
    } else if (controller1.ButtonDown.pressing()){
      blockingWizard.set(true);
    }

    //bkixjintkw for controlla doe bytton L1 L2
    if (controller1.ButtonL1.pressing()){
      intakeMotors.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } else if (controller1.ButtonL2.pressing()){
      intakeMotors.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    } else{ 
    intakeMotors.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);
    }

    //block Intake controlls for buttons R1 & R2 
    if (controller1.ButtonR1.pressing()){
      frontIntake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } else if (controller1.ButtonR2.pressing()){
      frontIntake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    } else{ 
      frontIntake.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);
    }
  
  /*
    if (controller1.ButtonA.pressing()) {

      double targetAngle;

      // Determine which direction to move
      if (isRaised) {
        targetAngle = -260.0;   // Move back down if it’s currently raised
      } else {
        targetAngle = 260.0;    // Move up if it’s currently down
      }

      // Start the motor movement and wait for completion
      intakeDeploy.spinFor(targetAngle, degrees);
    
      // --- Toggle the state for next time ---
      isRaised = !isRaised;

      // --- Debounce: wait until button is released ---
      while (controller1.ButtonA.pressing()) {
        vex::this_thread::sleep_for(20);
      }
    }
    */
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
