#include "robotconfig.h"


using namespace vex;

brain robotBrain;
controller controller1 = controller(primary);

//Movement motor constructors
motor left1Mtr = motor(PORT14, ratio6_1, true);
motor left2Mtr = motor(PORT13, ratio6_1, false);
motor left3Mtr = motor(PORT12, ratio6_1, false);
motor left4Mtr = motor(PORT11, ratio6_1, true);

motor_group leftMotors = motor_group(left1Mtr, left2Mtr, left3Mtr, left4Mtr);
motor right1Mtr = motor(PORT17, ratio6_1, false);
motor right2Mtr = motor(PORT18, ratio6_1, true);
motor right3Mtr = motor(PORT19, ratio6_1, true);
motor right4Mtr = motor(PORT20, ratio6_1, false);
motor_group rightMotors = motor_group(right1Mtr, right2Mtr, right3Mtr, right4Mtr);

motor leftStrafeMotor = motor(PORT15, ratio18_1, false);
motor rightStrafeMotor = motor(PORT16, ratio18_1, true);
motor_group strafeMotors = motor_group(leftStrafeMotor, rightStrafeMotor);

motor_group allMotors = motor_group(left1Mtr, left2Mtr, left3Mtr, left4Mtr, right1Mtr, right2Mtr, right3Mtr, right4Mtr);

//Objective motor constuctors
motor frontIntake = motor(PORT9, ratio6_1, false);
motor centerIntake = motor(PORT8, ratio18_1, false);
motor upperIntake = motor(PORT2, ratio18_1, false);
motor topIntake = motor(PORT4, ratio18_1, true);
motor_group intakeMotors = motor_group(frontIntake, centerIntake, upperIntake, topIntake);
motor_group noTopIntake = motor_group(frontIntake, centerIntake, upperIntake);

//Pnuematic constructors
//The hammer
digital_out unknownHammer = digital_out(robotBrain.ThreeWirePort.G);
//the block & scoring blocker
digital_out blockingWizard = digital_out(robotBrain.ThreeWirePort.H);

triport wireExpander = triport(PORT3);
// Create the digital_out instance on 3-Wire Port A
digital_out descore = digital_out(wireExpander.H);


//Sensor constructors
inertial inertialSensor = inertial(PORT21);
gps leftGpsSensor = gps(PORT6, 0, 0, mm, 0);
gps rightGpsSensor = gps(PORT8, 0, 0, mm, 0);
analog_in leftSideFront = analog_in(robotBrain.ThreeWirePort.C);
analog_in leftSideBack = analog_in(robotBrain.ThreeWirePort.A);
analog_in rightSideFront = analog_in(robotBrain.ThreeWirePort.D);
analog_in rightSideBack = analog_in(robotBrain.ThreeWirePort.B);
analog_in leftBack = analog_in(robotBrain.ThreeWirePort.F);
analog_in rightBack = analog_in(robotBrain.ThreeWirePort.E);

aivision::colordesc redObj(1, 177, 1, 47, 33, 0.6);    // Red color detection
aivision::colordesc blueObj(2, 12, 105, 173, 30.00, 0.39); // Blue color detection
aivision::colordesc greenObj(3, 155, 225, 68, 10.00, 0.20); // Green color detection
aivision::colordesc whiteObj(4, 255, 255, 255, 10.00, 0.20);
aivision::colordesc grayObj(5, 5, 5, 5, 20.00, 0.20);
aivision aiVisionE = aivision(PORT21, redObj, blueObj, greenObj, whiteObj, grayObj);

/*TURNING FUCNTIONS*/

void driveFowardWithHeading(double speed, double time, double kp, double kd) {

    double startHeading = rightGpsSensor.heading();
    double timer = 0;
    double previousError = 0;

    while (timer < time){
      //Read current heading on gps
      double currentHeading = rightGpsSensor.heading();

      //Compute heading error
      double headingError = startHeading - currentHeading;
      // Derivative term: change in error
      double derivative = (headingError - previousError) / 0.02; // 0.02 s loop time

      // Wrap to +- 180
      if (headingError > 180)  headingError -= 360;
      if (headingError < -180) headingError += 360;

      //proportional correction
      double adjustment = (kp * headingError) + (kd * derivative);
      
      leftMotors.spin(fwd, speed + adjustment, pct);
      rightMotors.spin(fwd, speed - adjustment, pct);

      wait(20, msec);
      timer += 20;
  
    }
  // Stop the left motor group using the 'brake' mode
  // 'brake' actively resists motion and stops the robot more quickly than 'coast'
  leftMotors.stop(brake);

  // Stop the right motor group using the same braking method
  rightMotors.stop(brake);
  
}

// Function to drive the robot forward at a specified speed and for a specified time
// Parameters:
//   speed - the velocity at which to drive (in percentage [-100, 100])
//   time  - the duration to drive (in milliseconds)
void driveFoward(int speed, int time) {
 
  // Spin the left motor group forward at the specified speed percentage
  // 'fwd' is the forward direction; 'pct' specifies the units as percent
  leftMotors.spin(fwd, speed, pct);

  // Spin the right motor group forward at the same speed
  // This ensures both sides move in unison for straight-line motion
  rightMotors.spin(fwd, speed, pct);

  // Wait for the specified time duration before stopping the motors
  // This is a blocking wait – the rest of the program halts here
  wait(time, msec);

  // Stop the left motor group using the 'brake' mode
  // 'brake' actively resists motion and stops the robot more quickly than 'coast'
  leftMotors.stop(brake);

  // Stop the right motor group using the same braking method
  rightMotors.stop(brake);
}



// Function to drive the robot in reverse at a specified speed and for a specified duration
// Parameters:
//   speed - the velocity at which to drive (in percentage [0 to 100])
//   time  - the duration to drive (in milliseconds)
void driveReverse(int speed, int time) {

  // Spin the left motor group in the reverse direction at the specified speed
  // 'reverse' indicates direction; 'pct' sets the speed units to percent
  leftMotors.spin(reverse, speed, pct);

  // Spin the right motor group in reverse at the same speed
  // Ensures both sides move together to drive straight backward
  rightMotors.spin(reverse, speed, pct);

  // Wait for the specified time duration while motors are spinning
  // This blocks execution of further code during the motion
  wait(time, msec);

  // Stop the left motor group using brake mode
  // 'brake' applies resistance, helping the robot stop quickly instead of coasting
  leftMotors.stop(brake);

  // Stop the right motor group in the same manner
  rightMotors.stop(brake);
}


/* MOVEMENT FUNCTIONS*/


//Vision functions

void alignBluePID(int speed, double kp, double kd, int centerPixel, int tolerance){
  // Take a snapshot and check if objects are detected
  int objects = aiVisionE.takeSnapshot(blueObj, 1);  // Only pass the greenObj descriptor
  // Get the center X and Y coordinates of the first detected object
  int centerX = aiVisionE.objects[0].centerX;
  int centerY = aiVisionE.objects[0].centerY;
  int prevError = 0;  // Store previous error for derivative term
  double error = centerPixel - centerX;
  double derivative = 0;
  double adjustment;
  printf("Object count: %d\n", aiVisionE.objectCount);
  while(centerX < centerPixel - tolerance || centerX > centerPixel + tolerance) {
    objects = aiVisionE.takeSnapshot(blueObj);
    centerX = aiVisionE.objects[0].centerX;
    centerY = aiVisionE.objects[0].centerY;
    error = centerPixel - centerX;
    derivative = error - prevError;
    if (error > 0){
        adjustment = speed + (error * kp) + (derivative * kd);
    } else if (error < 0){
        adjustment = -speed + (error * kp) + (derivative * kd);
    } else{
        adjustment = 0;
    }
    //Adjust position based on x cords of obj center
    //strafeMtr.spin(fwd, -adjustment, pct);
    wait(10, msec);
    if (objects > 0) {  
      centerX = aiVisionE.objects[0].centerX;
      printf("Object detected! Center X: %d Adjustment: %.2f\n", centerX, adjustment);
    } else {
      printf("No object detected!\n");
    }
  }
  printf("Obj found \n");
  printf("Center X: %.2f\n", centerX);
  //strafeMtr.stop(brake);
}

void alignRedPID(int speed, double kp, double kd, int centerPixel, int tolerance){
  // Take a snapshot and check if objects are detected
  int objects = aiVisionE.takeSnapshot(redObj, 1);  // Only pass the greenObj descriptor
  // Get the center X and Y coordinates of the first detected object
  int centerX = aiVisionE.objects[0].centerX;
  int centerY = aiVisionE.objects[0].centerY;
  int prevError = 0;  // Store previous error for derivative term
  double error = centerPixel - centerX;
  double derivative = 0;
  double adjustment;
  printf("Object count: %d\n", aiVisionE.objectCount);
  while(centerX < centerPixel - tolerance || centerX > centerPixel + tolerance) {
    objects = aiVisionE.takeSnapshot(redObj);
    centerX = aiVisionE.objects[0].centerX;
    centerY = aiVisionE.objects[0].centerY;
    error = centerPixel - centerX;
    derivative = error - prevError;
    if (error > 0){
        adjustment = speed + (error * kp) + (derivative * kd);
    } else if (error < 0){
        adjustment = -speed + (error * kp) + (derivative * kd);
    } else{
        adjustment = 0;
    }
    //Adjust position based on x cords of obj center
    //strafeMtr.spin(fwd, -adjustment, pct);
    wait(10, msec);
    if (objects > 0) {  
      centerX = aiVisionE.objects[0].centerX;
      printf("Object detected! Center X: %d Adjustment: %.2f\n", centerX, adjustment);
    } else {
      printf("No object detected!\n");
    }
  }
  printf("Obj found \n");
  printf("Center X: %.2f\n", centerX);
  //strafeMtr.stop(brake);
}

void alignGreenPID(int speed, double kp, double kd, int centerPixel, int tolerance){
  // Take a snapshot and check if objects are detected
  int objects = aiVisionE.takeSnapshot(greenObj, 1);  // Only pass the greenObj descriptor
  // Get the center X and Y coordinates of the first detected object
  int centerX = aiVisionE.objects[0].centerX;
  int centerY = aiVisionE.objects[0].centerY;
  int prevError = 0;  // Store previous error for derivative term
  double error = centerPixel - centerX;
  double derivative = 0;
  double adjustment;
  printf("Object count: %d\n", aiVisionE.objectCount);
  while(centerX < centerPixel - tolerance || centerX > centerPixel + tolerance) {
    objects = aiVisionE.takeSnapshot(greenObj);
    centerX = aiVisionE.objects[0].centerX;
    centerY = aiVisionE.objects[0].centerY;
    error = centerPixel - centerX;
    derivative = error - prevError;
    if (error > 0){
        adjustment = speed + (error * kp) + (derivative * kd);
    } else if (error < 0){
        adjustment = -speed + (error * kp) + (derivative * kd);
    } else{
        adjustment = 0;
    }
    //Adjust position based on x cords of obj center
    //strafeMtr.spin(fwd, adjustment, pct);
    wait(10, msec);
    if (objects > 0) {  
      centerX = aiVisionE.objects[0].centerX;
      printf("Object detected! Center X: %d Adjustment: %.2f\n", centerX, adjustment);
    } else {
      printf("No object detected!\n");
    }
  }
  printf("Goal found \n");
  printf("Center X: %.2f\n", centerX);
  //strafeMtr.stop(brake);
}


// Function: ztainConstantY_DirectionPositiveXRight
// Description:
//   Drives the robot in the positive X direction while using a PD controller 
//   to maintain a constant Y position.
//   GPS is used to measure the robot’s real-time X and Y position and heading.
//
// Parameters:
//   kp        - Proportional gain for correcting Y position
//   kd        - Derivative gain for correcting Y position
//   desiredY  - Target Y position to hold constant
//   desiredX  - Final X position to reach

void maintainConstantY_DirectionPositiveXRight(double kp, double kd, double desiredY, double desiredX, double speed) {

  // Initial sensor readings
  double ypos = rightGpsSensor.yPosition(mm);
  double xpos = rightGpsSensor.xPosition(mm);

  printf("x: %.1f  y: %.1f\n", xpos,ypos);

  // Y-axis control variables
  double yerror = ypos - desiredY;
  double previousYError = 0;
  double derivative = 0;
  int delayTime = 40;  // Loop delay in milliseconds
  int index = 0;

  // Drive right in X until reaching the target
  while (xpos < desiredX) {
    // Update current GPS data
    ypos = rightGpsSensor.yPosition(mm);
    xpos = rightGpsSensor.xPosition(mm);

    // Update Y error
    yerror = ypos - desiredY;

    // Calculate derivative only after the first loop
    if (index != 0) {
      derivative = yerror - previousYError;
    }

    // PD controller for Y-axis correction
    double adjustment = (kp * yerror) + (kd * (derivative / (delayTime / 1000.0)));
    previousYError = yerror;

    // Apply motor adjustments: adjust turning to maintain Y while moving right
    leftMotors.spin(fwd, speed + adjustment, pct);
    rightMotors.spin(fwd, speed - adjustment, pct);

    // Small delay for loop timing
    wait(delayTime, msec);

    ypos = rightGpsSensor.yPosition(mm);
    xpos = rightGpsSensor.xPosition(mm);  

    printf("# %4d x: %6.1f  y: %6.1f adj: %6.1f\n", index,xpos,ypos,adjustment);

    index++;
  }

  // Stop motors once target X is reached
  leftMotors.stop(brake);
  rightMotors.stop(brake);
}


// Function: maintainConstantY_DirectionNegativeXRight
// Description:
//   Drives the robot in the negative X direction while using a PD controller 
//   to maintain a constant Y position.
//   GPS is used to measure the robot’s real-time X and Y position and heading.
//
// Parameters:
//   kp        - Proportional gain for correcting Y position
//   kd        - Derivative gain for correcting Y position
//   desiredY  - Target Y position to hold constant
//   desiredX  - Final X position to reach

void maintainConstantY_DirectionNegativeXRight(double kp, double kd, double desiredY, double desiredX, double speed) {


  // Initial sensor readings
  double ypos = rightGpsSensor.yPosition(mm);
  double xpos = rightGpsSensor.xPosition(mm);

  // Y-axis control variables
  double yerror = ypos - desiredY;
  double previousYError = 0;
  double derivative = 0;
  int delayTime = 40;  // Loop delay in milliseconds
  int index = 0;

  // Drive left in X until reaching the target
  while (xpos > desiredX) {
    // Update current GPS data
    ypos = rightGpsSensor.yPosition(mm);
    xpos = rightGpsSensor.xPosition(mm);

    // Update Y error
    yerror = ypos - desiredY;

    // Calculate derivative only after the first loop
    if (index != 0) {
      derivative = yerror - previousYError;
    }

    // PD controller for Y-axis correction
    double adjustment = (kp * yerror) + (kd * (derivative / (delayTime / 1000.0)));
    previousYError = yerror;

    // Apply motor adjustments: adjust turning to maintain Y while moving left
    leftMotors.spin(fwd, speed - adjustment, pct);
    rightMotors.spin(fwd, speed + adjustment, pct);

    // Small delay for loop timing
    wait(delayTime, msec);
    index++;

    // Update current GPS data
    ypos = rightGpsSensor.yPosition(mm);
    xpos = rightGpsSensor.xPosition(mm);
  }

  // Stop motors once target X is reached
  leftMotors.stop(brake);
  rightMotors.stop(brake);
}



// Function: MaintainConstantX_DirectionPositiveYRight
// Description:
//   Drives the robot in the positive Y direction while using a PD controller 
//   to maintain a constant X position.
//   GPS is used to measure the robot’s real-time X and Y position and heading.
//
// Parameters:
//   kp        - Proportional gain for correcting X position
//   kd        - Derivative gain for correcting X position
//   desiredX  - Target X position to hold constant
//   desiredY  - Final Y position to reach

void maintainConstantX_DirectionPositiveYRight(double kp, double kd, double desiredX, double desiredY, double speed) {

  // Initial sensor readings
  double ypos = rightGpsSensor.yPosition(mm);
  double xpos = rightGpsSensor.xPosition(mm);

  printf("Parameters: %7.1f, %7.1f\n", desiredX, desiredY);

  // X-axis control variables
  double xerror = xpos - desiredX;
  double previousXError = 0;
  double derivative = 0;
  double delayTime = 40;  // Loop delay in milliseconds
  double adjustment;
  int index = 0;

  // Drive forward in Y until reaching the target
  while (ypos < desiredY) {
    // Update X error
    ypos = rightGpsSensor.yPosition(mm);
    xpos = rightGpsSensor.xPosition(mm);
    xerror = xpos - desiredX;


    // Calculate derivative only after the first loop
    if (index != 0) {
        derivative = xerror - previousXError;
    }

    // PD controller for X-axis correction
    adjustment = (kp * xerror) + (kd * (derivative / (delayTime / 1000.0)));
    previousXError = xerror;

    printf("%5d (%7.1f  %7.1f) %7.1f %7.1f\n", index, xpos, ypos, xerror, adjustment);


    // Apply motor adjustments: adjust turning to maintain X while moving forward
    leftMotors.spin(fwd, speed - adjustment, pct);
    rightMotors.spin(fwd, speed + adjustment, pct);

    // Small delay for loop timing
    wait(delayTime, msec);

    // Update current GPS data
    ypos = rightGpsSensor.yPosition(mm);
    xpos = rightGpsSensor.xPosition(mm);
    
    index++;
  }

  // Stop motors once target Y is reached
  leftMotors.stop(brake);
  rightMotors.stop(brake);

  printf("Terminated\n %5d (%7.1f  %7.1f) %7.1f %7.1f\n", index, xpos, ypos, xerror, adjustment);
}

// Function: maintainConstantX_DirectionNegativeYRight
// Description:
//   Drives the robot in the negative Y direction while using a PD controller 
//   to maintain a constant X position.
//   GPS is used to measure the robot’s real-time X and Y position and heading.
//
// Parameters:
//   kp        - Proportional gain for correcting X position
//   kd        - Derivative gain for correcting X position
//   desiredX  - Target X position to hold constant
//   desiredY  - Final Y position to reach

void maintainConstantX_DirectionNegativeYRight(double kp, double kd, double desiredX, double desiredY, double speed) {

  // Initial sensor readings
  double ypos = rightGpsSensor.yPosition(mm);
  double xpos = rightGpsSensor.xPosition(mm);
  

  // X-axis control variables
  double xerror = xpos - desiredX;
  double previousXError = 0;
  double derivative = 0;
  int delayTime = 40;  // Loop delay in milliseconds
  int index = 0;

  // Drive backward in Y until reaching the target
  while (ypos > desiredY) {
    // Update current GPS data
    ypos = rightGpsSensor.yPosition(mm);
    xpos = rightGpsSensor.xPosition(mm);
    

    // Update X error
    xerror = xpos - desiredX;

    // Calculate derivative only after the first loop
    if (index != 0) {
      derivative = xerror - previousXError;
    }

    // PD controller for X-axis correction
    double adjustment = (kp * xerror) + (kd * (derivative / (delayTime / 1000.0)));
    previousXError = xerror;

    // Apply motor adjustments: adjust turning to maintain X while moving backward
    leftMotors.spin(fwd, speed + adjustment, pct);
    rightMotors.spin(fwd, speed - adjustment, pct);

    // Small delay for loop timing
    wait(delayTime, msec);

    // Debug output
    /*
    printf("%f, %f\n", xpos, ypos);
    robotBrain.Screen.print("(%.2f, %.2f)  Heading: %.2f  X Error: %.2f  Adjustment: %.2f", 
                             xpos, ypos, heading, xerror, adjustment);
    robotBrain.Screen.newLine();
    */
    index++;

    // Update current GPS data
    ypos = rightGpsSensor.yPosition(mm);
    xpos = rightGpsSensor.xPosition(mm);
  }

  // Stop motors once target Y is reached
  leftMotors.stop();
  rightMotors.stop();
}

// Function: maintainConstantY_DirectionPositiveXLeft
// Description:
//   Drives the robot in the positive X direction while using a PD controller 
//   to maintain a constant Y position.
//   GPS is used to measure the robot’s real-time X and Y position and heading.
//
// Parameters:
//   kp        - Proportional gain for correcting Y position
//   kd        - Derivative gain for correcting Y position
//   desiredY  - Target Y position to hold constant
//   desiredX  - Final X position to reach

void maintainConstantY_DirectionPositiveXLeft(double kp, double kd, double desiredY, double desiredX, double speed) {
  
  // Initial sensor readings
  double ypos = leftGpsSensor.yPosition(mm);
  double xpos = leftGpsSensor.xPosition(mm);

  printf("Entering maintainConstantY_DirectionPositiveX\n");

  // Y-axis control variables
  double yerror = ypos - desiredY;
  double previousYError = 0;
  double derivative = 0;
  int delayTime =40;  // Loop delay in milliseconds
  int index = 0;

  // Drive right in X until reaching the target
  while (xpos < desiredX) {
    // Update current GPS data
    ypos = leftGpsSensor.yPosition(mm);
    xpos = leftGpsSensor.xPosition(mm);

    // Update Y error
    yerror = ypos - desiredY;

    // Calculate derivative only after the first loop
    if (index != 0) {
      derivative = yerror - previousYError;
    }

    // PD controller for Y-axis correction
    double adjustment = (kp * yerror) + (kd * (derivative / (delayTime / 1000.0)));
    previousYError = yerror;

    // Apply motor adjustments: adjust turning to maintain Y while moving right
    leftMotors.spin(fwd, speed + adjustment, pct);
    rightMotors.spin(fwd, speed - adjustment, pct);

    // Small delay for loop timing
    wait(delayTime, msec);

    ypos = leftGpsSensor.yPosition(mm);
    xpos = leftGpsSensor.xPosition(mm);  

    index++;
  }

  // Stop motors once target X is reached
  leftMotors.stop(brake);
  rightMotors.stop(brake);
}

// Function: maintainConstantY_DirectionNegativeXLeft
// Description:
//   Drives the robot in the negative X direction while using a PD controller 
//   to maintain a constant Y position.
//   GPS is used to measure the robot’s real-time X and Y position and heading.
//
// Parameters:
//   kp        - Proportional gain for correcting Y position
//   kd        - Derivative gain for correcting Y position
//   desiredY  - Target Y position to hold constant
//   desiredX  - Final X position to reach

void maintainConstantY_DirectionNegativeXLeft(double kp, double kd, double desiredY, double desiredX, double speed) {


  // Initial sensor readings
  double ypos = leftGpsSensor.yPosition(mm);
  double xpos = leftGpsSensor.xPosition(mm);

  // Y-axis control variables
  double yerror = ypos - desiredY;
  double previousYError = 0;
  double derivative = 0;
  int delayTime = 40;  // Loop delay in milliseconds
  int index = 0;

  // Drive left in X until reaching the target
  while (xpos > desiredX) {
    // Update current GPS data
    ypos = leftGpsSensor.yPosition(mm);
    xpos = leftGpsSensor.xPosition(mm);

    // Update Y error
    yerror = ypos - desiredY;

    // Calculate derivative only after the first loop
    if (index != 0) {
      derivative = yerror - previousYError;
    }

    // PD controller for Y-axis correction
    double adjustment = (kp * yerror) + (kd * (derivative / (delayTime / 1000.0)));
    previousYError = yerror;

    // Apply motor adjustments: adjust turning to maintain Y while moving left
    leftMotors.spin(fwd, speed - adjustment, pct);
    rightMotors.spin(fwd, speed + adjustment, pct);

    // Small delay for loop timing
    wait(delayTime, msec);
    index++;

    // Update current GPS data
    ypos = leftGpsSensor.yPosition(mm);
    xpos = leftGpsSensor.xPosition(mm);
  }

  // Stop motors once target X is reached
  leftMotors.stop(brake);
  rightMotors.stop(brake);
}

// Function: MaintainConstantX_DirectionPositiveYLeft
// Description:
//   Drives the robot in the positive Y direction while using a PD controller 
//   to maintain a constant X position.
//   GPS is used to measure the robot’s real-time X and Y position and heading.
//
// Parameters:
//   kp        - Proportional gain for correcting X position
//   kd        - Derivative gain for correcting X position
//   desiredX  - Target X position to hold constant
//   desiredY  - Final Y position to reach

void maintainConstantX_DirectionPositiveYLeft(double kp, double kd, double desiredX, double desiredY, double speed) {

  // Initial sensor readings
  double ypos = leftGpsSensor.yPosition(mm);
  double xpos = leftGpsSensor.xPosition(mm);

  printf("Parameters: %7.1f, %7.1f\n", desiredX, desiredY);


  // X-axis control variables
  double xerror = xpos - desiredX;
  double previousXError = 0;
  double derivative = 0;
  double  delayTime = 40;  // Loop delay in milliseconds
  int index = 0;
  double adjustment;

  // Drive forward in Y until reaching the target
  while (ypos < desiredY) {
    // Update X error
    ypos = leftGpsSensor.yPosition(mm);
    xpos = leftGpsSensor.xPosition(mm);
    xerror = xpos - desiredX;

    // Calculate derivative only after the first loop
    if (index != 0) {
        derivative = xerror - previousXError;
    }

    // PD controller for X-axis correction
    adjustment = (kp * xerror) + (kd * (derivative / (delayTime / 1000.0)));
    previousXError = xerror;

    // Apply motor adjustments: adjust turning to maintain X while moving forward
    leftMotors.spin(fwd, speed - adjustment, pct);
    rightMotors.spin(fwd, speed + adjustment, pct);

    // Small delay for loop timing
    wait(delayTime, msec);

    // Update current GPS data
    ypos = leftGpsSensor.yPosition(mm);
    xpos = leftGpsSensor.xPosition(mm);

    printf("%5d (%7.1f  %7.1f) %7.1f %7.1f\n", index, xpos, ypos, xerror, adjustment);

    
    index++;
  }

  // Stop motors once target Y is reached
  leftMotors.stop(brake);
  rightMotors.stop(brake);
  
}

// Function: maintainConstantX_DirectionNegativeYLeft
// Description:
//   Drives the robot in the negative Y direction while using a PD controller 
//   to maintain a constant X position.
//   GPS is used to measure the robot’s real-time X and Y position and heading.
//
// Parameters:
//   kp        - Proportional gain for correcting X position
//   kd        - Derivative gain for correcting X position
//   desiredX  - Target X position to hold constant
//   desiredY  - Final Y position to reach

void maintainConstantX_DirectionNegativeYLeft(double kp, double kd, double desiredX, double desiredY, double speed) {

  // Initial sensor readings
  double ypos = leftGpsSensor.yPosition(mm);
  double xpos = leftGpsSensor.xPosition(mm);
  

  // X-axis control variables
  double xerror = xpos - desiredX;
  double previousXError = 0;
  double derivative = 0;
  int delayTime = 40;  // Loop delay in milliseconds
  int index = 0;

  // Drive backward in Y until reaching the target
  while (ypos > desiredY) {
    // Update current GPS data
    ypos = leftGpsSensor.yPosition(mm);
    xpos = leftGpsSensor.xPosition(mm);
    

    // Update X error
    xerror = xpos - desiredX;

    // Calculate derivative only after the first loop
    if (index != 0) {
      derivative = xerror - previousXError;
    }

    // PD controller for X-axis correction
    double adjustment = (kp * xerror) + (kd * (derivative / (delayTime / 1000.0)));
    previousXError = xerror;

    // Apply motor adjustments: adjust turning to maintain X while moving backward
    leftMotors.spin(fwd, speed + adjustment, pct);
    rightMotors.spin(fwd, speed - adjustment, pct);

    // Small delay for loop timing
    wait(delayTime, msec);

    // Debug output
    /*
    printf("%f, %f\n", xpos, ypos);
    robotBrain.Screen.print("(%.2f, %.2f)  Heading: %.2f  X Error: %.2f  Adjustment: %.2f", 
                             xpos, ypos, heading, xerror, adjustment);
    robotBrain.Screen.newLine();
    */
    index++;

    // Update current GPS data
    ypos = leftGpsSensor.yPosition(mm);
    xpos = leftGpsSensor.xPosition(mm);
  }

  // Stop motors once target Y is reached
  leftMotors.stop();
  rightMotors.stop();
}



// Function to rotate the robot clockwise to a specified angle using PD control with a limited I-term
void rotateInertialClockWisePID(double angle, double kp, double kd, double ki) {
  // Initialize a small buffer to store the last 5 error terms for integral action
  const int bufferSize = 5;
  double errorBuffer[bufferSize] = {0, 0, 0, 0, 0};
  int bufferIndex = 0;

  // Initialize the error index counter for debugging purposes
  int errorIndex = 0;

  // Set the initial target error from the desired angle
  double angleError = angle;
  double lastAngleError = angle;  // Used for derivative calculation

  // Variable to store the current heading from the inertial sensor
  double inertialHeading = 0;

  // Reset the inertial sensor's heading to zero
  inertialSensor.setHeading(0, degrees);

  // Spin the motors to start the rotation: left motor forward, right motor backward
  leftMotors.spin(fwd, 100, pct);
  rightMotors.spin(reverse, 100, pct);

  // Wait 100 milliseconds for motion to stabilize
  wait(100, msec);

  // Control loop: You can adjust the loop condition as required (e.g., use a loop with time-out or condition based on error)
  for (int i = 0; i < 200; i++) {
      // Calculate derivative (difference between current and last error divided by loop time ~10ms)
      double derivative = (angleError - lastAngleError) / 0.01;
      lastAngleError = angleError;

      // Update the error buffer with the current error using a circular buffer mechanism
      errorBuffer[bufferIndex] = angleError;
      bufferIndex = (bufferIndex + 1) % bufferSize;
      
      // Sum the errors in the buffer
      double errorSum = 0;
      for (int j = 0; j < bufferSize; j++) {
          errorSum += errorBuffer[j];
      }

      // Calculate motor speed using PD control and add I component
      double motorSpeed = kp * angleError + kd * derivative + ki * errorSum;

      // Set the motor speeds
      leftMotors.spin(fwd, motorSpeed, pct);
      rightMotors.spin(fwd, -motorSpeed, pct);

      // Wait 10 milliseconds
      wait(10, msec);

      // Update the inertial sensor heading
      inertialHeading = inertialSensor.heading();

      // Recalculate the error (error decreases as heading approaches target)
      angleError = angle - inertialHeading;

      // Increment errorIndex (for debugging purposes)
      errorIndex++;
      // For debugging: printf("%d, %.1f;\n", 10 * errorIndex, inertialSensor.heading());
  }

  // Stop motors once the rotation is complete
  leftMotors.stop(brake);
  rightMotors.stop(brake);

  
  // wait(1000, msec);
  // For debugging: printf("Final angle: %.1f;\n", inertialSensor.heading());
}

// Function to rotate the robot counterclockwise to a specified angle using PD + limited I control
void rotateInertialCounterClockWisePID(double angle, double kp, double kd, double ki) {
  // Initialize the error buffer to hold the last 5 error values
  const int bufferSize = 5;
  double errorBuffer[bufferSize] = {0, 0, 0, 0, 0};
  int bufferIndex = 0;

  // Debugging variable to count iterations
  int errorIndex = 0;

  // Initialize the error values
  double angleError = angle;
  double lastAngleError = angle;

  // Variable for current heading
  double inertialHeading = 0;

  // Reset inertial sensor
  inertialSensor.setHeading(0, degrees);

  // Start rotation: left motor backward, right motor forward
  leftMotors.spin(reverse, 100, pct);
  rightMotors.spin(fwd, 100, pct);

  // Small delay to stabilize
  wait(100, msec);

  // Run control loop up to 100 iterations
  for (int i = 0; i < 200; i++) {
      // Update inertial heading
      inertialHeading = inertialSensor.heading();

      // Calculate error for counterclockwise (angle - (360 - heading))
      angleError = angle - (360 - inertialHeading);

      // Derivative term
      double derivative = (angleError - lastAngleError) / 0.01;
      lastAngleError = angleError;

      // Store error in circular buffer
      errorBuffer[bufferIndex] = angleError;
      bufferIndex = (bufferIndex + 1) % bufferSize;

      // Sum last 5 errors for limited integral term
      double errorSum = 0;
      for (int j = 0; j < bufferSize; j++) {
          errorSum += errorBuffer[j];
      }

      // Compute motor speed using PID (with limited I)
      double motorSpeed = kp * angleError + kd * derivative + ki * errorSum;

      // Set motor speeds for counterclockwise rotation
      leftMotors.spin(fwd, -motorSpeed, pct);
      rightMotors.spin(fwd, motorSpeed, pct);

      // Delay between control loop iterations
      wait(10, msec);

      // Optional debug print
      errorIndex++;
      // printf("%d, %.1f;\n", 10 * errorIndex, inertialSensor.heading());
  }

  // Stop motors when done
  leftMotors.stop(brake);
  rightMotors.stop(brake);
}




// Function: backStraightToNegativeXPosition
// Purpose: Drives the robot backward in a straight line at 10% duty cycle
//          until the left-side GPS sensor detects that the robot has reached
//          a specified negative x-position (in inches), or until a timeout occurs.
//
// Parameters:
//    xPosition - the target negative x-coordinate (in inches) at which to stop

void forwardStraightToPositiveXPosition(double xPosition) {
  // Start both left and right motors in reverse at 10% duty cycle
  leftMotors.spin(forward, 20, percent);
  rightMotors.spin(forward, 20, percent);

  // Start a timer
  int elapsedTime;             // in milliseconds
  const int timeout = 1000;        // 2 seconds
  int startTime = robotBrain.timer(msec);

  // Continuously check the x-position from the right GPS sensor
  while (rightGpsSensor.xPosition(mm) > xPosition) {
    elapsedTime = robotBrain.timer(msec) - startTime;

    if (elapsedTime > timeout){
      // timeout hit - stop the loop
      break;
    }
    wait (20, msec);
  }



  // Stop both motors (either on success or timeout)
  leftMotors.stop();
  rightMotors.stop();
}




void forwardStraightToNegativeXPosition(double xPosition) {
  // Start both left and right motors in reverse at 10% duty cycle
  leftMotors.spin(forward, 20, percent);
  rightMotors.spin(forward, 20, percent);

  // Start a timer
  int elapsedTime;             // in milliseconds
  const int timeout = 1000;        // 2 seconds
  int startTime = robotBrain.timer(msec);

  // Continuously check the x-position from the right GPS sensor
  while (rightGpsSensor.xPosition(mm) > xPosition) {
    elapsedTime = robotBrain.timer(msec) - startTime;

    if (elapsedTime > timeout){
      // timeout hit - stop the loop
      break;
    }
    wait (20, msec);
  }



  // Stop both motors (either on success or timeout)
  leftMotors.stop();
  rightMotors.stop();
}

// Function: trackToCenterRedLeft
// Description:
//   Drives the robot forward while comparing -X and Y coordinates from the right GPS sensor.
//   Adjusts motor speeds using PD control based on the difference between -X and Y.
//   If -X > Y: speed up right, slow left.
//   If -X < Y: speed up left, slow right.
//   If -X == Y: drive forward at 50%.

void trackToCenterRedLeft(double kp, double kd) {
  
  // Initial readings
  double ypos = rightGpsSensor.yPosition(mm) + 100;
  double xpos = rightGpsSensor.xPosition(mm);

  // Control variables
  double error = 0;
  double previousError = 0;
  double derivative = 0;
  double adjustment = 0;
  const double delayTime = 40; // ms
  int index = 0;

  // Drive forward while Y position is greater than 400 mm
  while (ypos > 400) {
    // Read current GPS values
    ypos = rightGpsSensor.yPosition(mm) - 100;
    xpos = rightGpsSensor.xPosition(mm);

    double negX = -xpos;

    // Compute error between -X and Y
    error = negX - ypos;

    // Check if robot is at center (negX == Y)
    if (fabs(error) < 1.0) {  // small threshold for equality
      leftMotors.spin(fwd, 25, pct);
      rightMotors.spin(fwd, 25, pct);
    } else {
      // Calculate derivative term
      if (index != 0) {
        derivative = (error - previousError) / (delayTime / 1000.0);
      }

      // PD adjustment
      adjustment = (kp * error) + (kd * derivative);

      if (error > 0) {
        // -X > Y: Speed up right, slow down left
        leftMotors.spin(fwd, 25 - adjustment, pct);
        rightMotors.spin(fwd, 25 + adjustment, pct);
      } else {
        // -X < Y: Speed up left, slow down right
        leftMotors.spin(fwd, 25 + adjustment, pct);
        rightMotors.spin(fwd, 25 - adjustment, pct);
      }

      previousError = error;
    }

    // Debug print
    printf("%5d (X: %7.1f, Y: %7.1f, Error: %7.1f, Adj: %7.1f)\n", index, xpos, ypos, error, adjustment);

    // Wait and increment loop
    wait(delayTime, msec);
    index++;
  }

  // Stop motors once loop condition ends
  leftMotors.stop(brake);
  rightMotors.stop(brake);

  printf("Terminated\n");
}


// Function: trackToCenterBlueLeft
// Description:
//   Drives the robot forward while comparing X and -Y coordinates from the right GPS sensor.
//   Adjusts motor speeds using PD control based on the difference between X and -Y.
//   If X > -Y: speed up right, slow left.
//   If X < -Y: speed up left, slow right.
//   If X == -Y: drive forward at 50%.

void trackToCenterBlueLeft(double kp, double kd, double offset) {

    // Initial GPS readings
    double xpos = rightGpsSensor.xPosition(mm);
    double ypos = rightGpsSensor.yPosition(mm) -160;

    // Control variables
    double error = 0;
    double previousError = 0;
    double derivative = 0;
    double adjustment = 0;
    const double delayTime = 40; // milliseconds
    int index = 0;

    // Continue while robot is above target Y (in negative Y direction)
    while (xpos > 500) {
        // Update GPS readings
        xpos = rightGpsSensor.xPosition(mm);
        ypos = rightGpsSensor.yPosition(mm) - offset;

        double negY = -ypos;  // Use -Y for comparison

        // Compute error between X and -Y
        error = xpos - negY;

        // Calculate derivative term
        if (index != 0) {
            derivative = (error - previousError) / (delayTime / 1000.0);
        }

        // PD control
        adjustment = (kp * error) + (kd * derivative);

        leftMotors.spin(forward, 50 - adjustment, pct);
        rightMotors.spin(forward, 50 + adjustment, pct);

        previousError = error;


        // Debug output
        printf("%5d (X: %7.1f, Y: %7.1f, Error: %7.1f, Adj: %7.1f)\n", index, xpos, ypos, error, adjustment);

        wait(delayTime, msec);
        index++;
    }
 
    // Stop the robot
    leftMotors.stop(brake);
    rightMotors.stop(brake);

    printf("Terminated\n");
}

void unknownHammerDown(){
    unknownHammer.set(true);
}
void unknownHammerUp(){
    unknownHammer.set(false);
}

void scoreAllow(){
    blockingWizard.set(true);
}
void scoreBlock(){
    blockingWizard.set(false);
}
/*
void shakeAndIntake(int cycles){  
    for (int count = 0; count < cycles; count++) {
        // Move in reverse
        leftStrafeMotor.stop(hold);
        rightStrafeMotor.stop(hold);
        intakeToTop.spin(fwd, 100, pct);
        allMotors.spin(reverse, 40, pct);
        wait(150, msec);

        // Move forward
        allMotors.spin(forward, 40, pct);
        wait(200, msec);
    }
    allMotors.stop(hold);
    wait(300, msec);
    intakeToTop.stop();
}
*/
void ramAndIntakeSkills(int cycles){  
    for (int count = 0; count < cycles; count++) {
        // Move in reverse
        leftStrafeMotor.stop(hold);
        rightStrafeMotor.stop(hold);
        intakeMotors.spin(fwd, 100, pct);
        allMotors.spin(fwd, 100, pct);
        wait(350, msec);

        // Move forward
        allMotors.stop(brake);
        wait(200, msec);
    }
    allMotors.stop(hold);
    intakeMotors.stop(coast);
    noTopIntake.spin(fwd, 100, pct);
    wait(500, msec);
    noTopIntake.stop(coast);
  }

  void ramAndIntakeAuton(int cycles){  
    for (int count = 0; count < cycles; count++) {
        // Move in reverse
        leftStrafeMotor.stop(hold);
        rightStrafeMotor.stop(hold);
        frontIntake.spin(fwd, 100, pct);
        centerIntake.spin(fwd, 100, pct);
        allMotors.spin(fwd, 100, pct);
        wait(500, msec);

        // Move forward
        allMotors.stop(coast);
        wait(400, msec);
    }
    allMotors.stop(hold);
    centerIntake.stop(brake);
    driveReverse(30, 300);
    frontIntake.spin(fwd, 100, pct);
    centerIntake.spin(fwd, 100, pct);
    wait(500, msec);
    frontIntake.stop(coast);
    centerIntake.stop(coast);
  }

void driveLeftWithH(double speed, double time){
  leftStrafeMotor.spin(fwd, speed, pct);
  rightStrafeMotor.spin(reverse, speed, pct);

  wait(time, msec);

  leftStrafeMotor.stop(hold);
  rightStrafeMotor.stop(hold);
}


void scoringTopSkills(double speed, double time){
  intakeMotors.spin(fwd, 100, pct);
  wait(time, msec);
  intakeMotors.stop(coast);
}

void leftTurnAlign(){
  double currentHeading = leftGpsSensor.heading();
  double desiredHead = 180.0;
  if (currentHeading < 180){
    double robotTurn = desiredHead - currentHeading;
    rotateInertialClockWisePID(robotTurn, .40, .02, 0);
  }
  else if (currentHeading > 180){
    double robotTurn = currentHeading - desiredHead;
    rotateInertialCounterClockWisePID(robotTurn, .40, .02, 0);
  }
}

void rightTurnAlign(){
  double currentHeading = rightGpsSensor.heading();
  double desiredHead = 0.0;
  double desiredH = 359.0;
  if (currentHeading > 300){
    double robotTurn = desiredH - currentHeading;
    rotateInertialClockWisePID(robotTurn, .40, .02, 0);
  }
  else if (currentHeading > 0){
    double robotTurn = currentHeading;
    rotateInertialCounterClockWisePID(robotTurn, .40, .02, 0);
  }
}

void parkingTurn(){
  double currentHeading = rightGpsSensor.heading();
  double desiredHead = 0.0;
  double desiredH = 270.0;
  if (currentHeading > 300){
    double robotTurn = currentHeading - desiredH;
    rotateInertialClockWisePID(robotTurn, .40, .02, 0);
  }
  else if (currentHeading > 0){
    double robotTurn = currentHeading + 90;
    rotateInertialCounterClockWisePID(robotTurn, .40, .02, 0);
  }
}
/*
void strafeWithDistanceL(float desiredD, float kp){
    //driveLeftWithH(1200, 80);
    int delayTime = 50; 
    int timeOut = 1000;
    int start = 0;
    double tol = 5.0;
    //Error and stuff
    float lsf = distanceMM(leftSideFront);
    float lsb = distanceMM(leftSideBack); // - 19.05
    float pError;
    float correction; //abcdefghijklmnopqrstuvwxyz

    while(start < timeOut){
      //Reads values from distance sensors
      lsf = distanceMM(leftSideFront);
      lsb = distanceMM(leftSideBack);
      float distance = fmin(lsf, lsb);
      //Averge from the values give the error
      pError = desiredD - distance;
      printf(" | pError = %.2f\n | lsf = %.2f | lsb = %.2f | dist = %.2f", pError, lsf, lsb, distance);
      correction = 2 * (pError) * kp;

      if (fabs(pError) < tol ) break;

      if (correction > 80) correction = 80;
      if (correction < -80) correction = -80;

      leftStrafeMotor.spin(reverse, correction , pct);
      rightStrafeMotor.spin(fwd, correction, pct);

      wait(delayTime, msec);
      start += delayTime;
    }

    leftStrafeMotor.stop(hold);
    rightStrafeMotor.stop(hold);
  }
*/

  void strafeWithDistanceL(float desiredD, float kp){
    //driveLeftWithH(1200, 80);
    int delayTime = 50; 
    int timeOut = 700;
    int start = 0;
    const double offset = 50.0;
    //Error and stuff
    float lsf = distanceMM(leftSideFront);
    float lsb = distanceMM(leftSideBack) - offset; 
    float pError;
    float correction; //abcdefghijklmnopqrstuvwxyz

    while(start < timeOut){
      //Reads values from distance sensors
      lsf = distanceMM(leftSideFront);
      lsb = distanceMM(leftSideBack) - offset;

      float distance = fmin(lsf, lsb);

      //Averge from the values give the error
      pError = desiredD - distance;

      printf(" | pError = %.2f\n | lsf = %.2f | lsb = %.2f | dist = %.2f\n", pError, lsf, lsb, distance);

      if (pError <= 10 && pError >= -10) {
        correction = kp * pError * 0.2;
      }
      
      else {

        if (pError > 0)
          correction = kp * (pError - 10);
        else
          correction = kp * (pError + 10);

        if (correction > 80) correction = 80;
        if (correction < -80) correction = -80;
      }

      leftStrafeMotor.spin(reverse, correction, pct);
      rightStrafeMotor.spin(fwd, correction, pct);
       
      wait(delayTime, msec);
      start += delayTime;
    }
    leftStrafeMotor.stop(hold);
    rightStrafeMotor.stop(hold);
}

 

  void strafeWithDistanceR(float desiredD, float kp){
     //driveLeftWithH(1200, 80);
    int delayTime = 50; 
    int timeOut = 700;
    int start = 0;
    const double offset = 50.0;
    //Error and stuff
    float rsf = distanceMM(rightSideFront);
    float rsb = distanceMM(rightSideBack) - offset; 
    float pError;
    float correction; //abcdefghijklmnopqrstuvwxyz

    while(start < timeOut){
      //Reads values from distance sensors
      rsf = distanceMM(rightSideFront);
      rsb = distanceMM(rightSideBack) - offset;

      float distance = fmin(rsf,rsb);

      //Averge from the values give the error
      pError = desiredD - distance;

      printf(" | pError = %.2f\n | rsf = %.2f | rsb = %.2f | dist = %.2f\n", pError, rsf, rsb, distance);

      if (pError <= 10 && pError >= -10) {
        correction = kp * pError * 0.2;
      }
      
      else {
        if (pError > 0)
          correction = kp * (pError - 10);
        else
          correction = kp * (pError + 10);

        if (correction > 80) correction = 80;
        if (correction < -80) correction = -80;
      }

      leftStrafeMotor.spin(fwd, correction, pct);
      rightStrafeMotor.spin(reverse, correction, pct);

      wait(delayTime, msec);
      start += delayTime;
    }

    leftStrafeMotor.stop(hold);
    rightStrafeMotor.stop(hold);
  }

  void alignWithDistanceR(float desiredD, double kp, int speed){
      float rb = distanceMM(rightBack);
      float pError;
      float correction;
      float delayTime = 40;
      double tol = 10.0;

      int timeOut = 1000;
      int start = 0;

      while(start < timeOut){
          rb = distanceMM(rightBack);
          printf("RB = %.2f\n", rb);
          pError = desiredD - rb;

          //correction = pError * kp;
          if (fabs(pError) < tol) break;

          leftMotors.spin(fwd, speed, pct);
          rightMotors.spin(fwd, speed, pct);

          wait(delayTime, msec);
          start += delayTime;
      }

      leftMotors.stop();
      rightMotors.stop();
  }

  void alignWithDistanceL(float desiredD, double kp){
      float rb = distanceMM(rightBack);
      float pError;
      float correction;
      float delayTime = 40;

      int timeOut = 1000;
      int start = 0;

      while(start < timeOut){
          rb = distanceMM(rightBack);
          printf("LB = %.2f\n", rb);
          pError = desiredD - rb;

          //correction = pError * kp;
          if (fabs(pError) < 5) break;

          leftMotors.spin(fwd, 50, pct);
          rightMotors.spin(fwd, 50, pct);

          wait(delayTime, msec);
      }

      leftMotors.stop(hold);
      rightMotors.stop(hold);
  }

  /*
  void AlignLeftSide(){

    const double tol = 30.0;      // mm difference allowed between RSF and RSB
    const int stepMs = 20;
    const int timeoutMs = 1000;

    // Tune these
    const double kp = 0.05;        // pct per mm
    const double kd = 0.00;        // pct per (mm per sec)

    const int maxPct = 20;
    const int minPct = 2;

    double lastErr = 0.0;
    int elapsed = 0;

    while (elapsed < timeoutMs)
    {
        double lsf = distanceMM(leftSideFront);
        double lsb = distanceMM(leftSideBack) - 50;

        double err = (lsf - lsb);   // >0 means front is farther than back (or vice versa)

        // stop if aligned
        if (fabs(err) < tol){
          leftMotors.stop(brake);
          rightMotors.stop(brake);
          break;
        }

        // derivative (scaled)
        double derr = (err - lastErr) / (stepMs / 1000.0);
        lastErr = err;

        double turn = kp * err + kd * derr;

        // clamp
        if (turn >  maxPct) turn =  maxPct;
        if (turn < -maxPct) turn = -maxPct;

        // apply turn (sign may need flipping depending on your drivetrain)
        leftMotors.stop(hold);
        rightMotors.spin(reverse, turn, pct);

        wait(stepMs, msec);
        elapsed += stepMs;
    }

    leftMotors.stop(brake);
    rightMotors.stop(brake);
}
*/
void AlignLeftSide(){

    const double tol = 3.0;
    const int stepMs = 10;
    const int timeoutMs = 500;

    const double kp = 0.06;
    const double kd = 0.02;
    const double maxPct = 18;
    const double offsetMM = 54.0;

    int elapsed = 0;
    int withinTolCount = 0;

    double prevErr = 0.0;
    double prevTurn = 0.0;

    leftMotors.setMaxTorque(40, percent);
    rightMotors.setMaxTorque(40, percent);

    while (elapsed < timeoutMs)
    {
        double lsf = distanceMM(leftSideFront);
        double lsb = distanceMM(leftSideBack) - offsetMM;

        double err = (lsf - lsb) / 200.0;

        if (fabs(err) < tol)
        {
            withinTolCount++;
            if (withinTolCount >= 12)
            {
                break;
            }

            leftMotors.spin(forward, 0, pct);
            rightMotors.spin(forward, 0, pct);

            wait(stepMs, msec);
            elapsed += stepMs;
            prevErr = err;
            prevTurn = 0;
            continue;
        }

        withinTolCount = 0;

        double derr = (err - prevErr) / (stepMs / 1000.0);

        double turn = kp * err + kd * derr;

        double scale = fmin(1.0, fabs(err) / 60.0);
        turn *= scale;

        if (turn >  maxPct) turn =  maxPct;
        if (turn < -maxPct) turn = -maxPct;

        turn = 0.5 * prevTurn + 0.5 * turn;

        leftMotors.spin(forward, -turn, pct);
        rightMotors.spin(forward,  turn, pct);

        prevErr = err;
        prevTurn = turn;

        wait(stepMs, msec);
        elapsed += stepMs;
    }

    leftMotors.stop(hold);
    rightMotors.stop(hold);
}
/*
void AlignLeftSide(){

    const double tol = 10.0;
    const int stepMs = 10;
    const int timeoutMs = 1000;

    const double kp = 0.20;
    const double kd = 0.1;   
    const int maxPct = 20;
    const double offsetMM = 54.0;

    int elapsed = 0;
    int withinTolCount = 0;
    int loopCount = 0;

    double prevErr = 0.0;

    leftMotors.setMaxTorque(40, percent);
    rightMotors.setMaxTorque(40, percent);

    while (elapsed < timeoutMs)
    {
        double lsf = distanceMM(leftSideFront);
        double lsb = distanceMM(leftSideBack) - offsetMM;

        double err = lsf - lsb;
        double turn = 0.0;

        if (fabs(err) < tol)
        {
            withinTolCount++;

            if (withinTolCount >= 5)
            {
                printf("loop=%d  lsf=%.2f  lsb=%.2f  err=%.2f  turn=%.2f\n",
                       loopCount, lsf, lsb, err, turn);
                break;
            }
        }
        else
        {
            withinTolCount = 0;

            double derr = err - prevErr;

            turn = kp * err + kd * derr;
        }

        if (turn >  maxPct) turn =  maxPct;
        if (turn < -maxPct) turn = -maxPct;

        printf("loop=%d  lsf=%.2f  lsb=%.2f  err=%.2f  turn=%.2f\n",
               loopCount, lsf, lsb, err, turn);

        leftMotors.spin(forward, -turn, pct);
        rightMotors.spin(forward,  turn, pct);

        prevErr = err;

        wait(stepMs, msec);
        elapsed += stepMs;
        loopCount++;
    }

    leftMotors.stop(brake);
    rightMotors.stop(brake);
}
*/

/*
void AlignRightSide(){
    const double tol = 20.0;      // mm difference allowed between RSF and RSB
    const int stepMs = 40;
    const int timeoutMs = 1000;

    // Tune these
    const double kp = .40;        // pct per mm
    const double kd = 0.02;        // pct per (mm per sec)

    const int maxPct = 30;
    const int minPct = 8;

    double lastErr = 0.0;
    int elapsed = 0;

    while (elapsed < timeoutMs)
    {
        double rsf = distanceMM(rightSideFront);
        double rsb = distanceMM(rightSideBack) - 19.05;

        double err = (rsf - rsb);   // >0 means front is farther than back (or vice versa)

        // stop if aligned
        if (fabs(err) <= tol) break;

        // derivative (scaled)
        double derr = (err - lastErr) / (stepMs / 1000.0);
        lastErr = err;

        double turn = kp * err + kd * derr;

        // clamp
        if (turn >  maxPct) turn =  maxPct;
        if (turn < -maxPct) turn = -maxPct;

        // minimum power so it actually moves
        if (turn > 0 && turn <  minPct) turn =  minPct;
        if (turn < 0 && turn > -minPct) turn = -minPct;

        // apply turn (sign may need flipping depending on your drivetrain)
        leftMotors.spin(fwd,  turn, pct);
        rightMotors.spin(reverse, turn, pct);

        wait(stepMs, msec);
        elapsed += stepMs;
    }

    leftMotors.stop(brake);
    rightMotors.stop(brake);
}
*/



void testDist() {
  while (true) {
    // 12-bit ADC counts (0-4095)
    int32_t LSF_ADC = rightSideFront.value(vex::analogUnits::range12bit);
    int32_t LSB_ADC = rightSideBack.value(vex::analogUnits::range12bit);

    // Convert ADC -> mV (0-3300 mV)
    double LSF_mV = (double)LSF_ADC * 5000.0 / 4095.0;
    double LSB_mV = (double)LSB_ADC * 5000.0 / 4095.0;

    // Convert mV -> mm (your polynomial)
    double LF_mm = (-1.504e-13 * pow(LSF_mV, 5.0)
                  + 1.243e-9  * pow(LSF_mV, 4.0)
                  -4.0513e-6  * pow(LSF_mV, 3.0)
                  + 6.599e-3  * pow(LSF_mV, 2.0)
                  -5.561   * LSF_mV
                  + 2197.0);

    double RB_mm = (-1.504e-13 * pow(LSB_mV, 5.0)
                  + 1.243e-9  * pow(LSB_mV, 4.0)
                  -4.0513e-6  * pow(LSB_mV, 3.0)
                  + 6.599e-3  * pow(LSB_mV, 2.0)
                  -5.561   * LSB_mV
                  + 2197.0); 

    printf("RSB: %.2f mm | RSF: %.2f mm\n", RB_mm, LF_mm);
    //printf("LSF: %ld ADC (%.1f mV) | LSB: %ld ADC (%.1f mV)\n",
    //       (long)LSF_ADC, LSF_mV, (long)LSB_ADC, LSB_mV);

    vex::wait(10000, vex::msec);
  }
}

/*
double distanceMM(vex::analog_in& sensor){
  // Read 12-bit ADC counts (0–4095)
  int32_t adc = sensor.value(vex::analogUnits::range12bit);

  // Convert ADC -> mV (assumes 0–5000 mV reference)
  double mV = (double)adc * 5000.0 / 4095.0;

  // Convert mV -> mm (your polynomial)
  double mm =
      (-1.504e-13 * pow(mV, 5.0)) +
      ( 1.243e-9  * pow(mV, 4.0)) +
      ( -4.0513e-6  * pow(mV, 3.0)) +
      ( 6.599e-3  * pow(mV, 2.0)) +
      ( -5.561     * mV) +
      ( 2197.0);

  return mm;
}
*/

double distanceMM(vex::analog_in& sensor){
    const int numSamples = 5;
    int32_t adcSamples[numSamples];

    for (int i = 0; i < numSamples; i++)
    {
        adcSamples[i] = sensor.value(vex::analogUnits::range12bit);
        wait(2, msec);
    }

    // Manual bubble sort for 5 elements
    for (int i = 0; i < numSamples - 1; i++)
    {
        for (int j = 0; j < numSamples - 1 - i; j++)
        {
            if (adcSamples[j] > adcSamples[j + 1])
            {
                int32_t temp = adcSamples[j];
                adcSamples[j] = adcSamples[j + 1];
                adcSamples[j + 1] = temp;
            }
        }
    }

    int32_t adcMedian = adcSamples[2];

    double mV = (double)adcMedian * 5000.0 / 4095.0;

    double mm =
        (-1.504e-13 * pow(mV, 5.0)) +
        ( 1.243e-9  * pow(mV, 4.0)) +
        (-4.0513e-6 * pow(mV, 3.0)) +
        ( 6.599e-3  * pow(mV, 2.0)) +
        (-5.561      * mV) +
        ( 2197.0);

    return mm;
}

/*
void AlignLeftSide()
{
    const double tol = 20.0;      // mm difference allowed between LSF and LSB
    const int stepMs = 20;
    const int timeoutMs = 2000;

    // Tune these
    const double kp = 0.40;        // pct per mm
    const double kd = 0.02;        // pct per (mm per sec)

    const int maxPct = 30;
    const int minPct = 8;

    double lastErr = 0.0;
    int elapsed = 0;

    while (elapsed < timeoutMs)
    {
        double lsf = distanceMM(leftSideFront);
        double lsb = distanceMM(leftSideBack) - 19.05;

        // Positive error means front is farther from wall than back
        double err = (lsb - lsf);

        // stop if aligned
        if (fabs(err) <= tol)
            break;

        // derivative term
        double derr = (err - lastErr) / (stepMs / 1000.0);
        lastErr = err;

        double turn = kp * err + kd * derr;

        // clamp magnitude
        if (turn >  maxPct) turn =  maxPct;
        if (turn < -maxPct) turn = -maxPct;

        // enforce minimum magnitude
        if (turn > 0 && turn <  minPct) turn =  minPct;
        if (turn < 0 && turn > -minPct) turn = -minPct;

        // -----------------------------
        // Explicit direction handling
        // -----------------------------
        if (turn > 0)
        {
            // TURN CCW (mirror of right-side logic)
            leftMotors.spin(reverse, turn, pct);
            rightMotors.spin(fwd,    turn, pct);
        }
        else
        {
            // TURN CW
            leftMotors.spin(fwd,  -turn, pct);
            rightMotors.spin(reverse, -turn, pct);
        }

        wait(stepMs, msec);
        elapsed += stepMs;
    }

    leftMotors.stop(hold);
    rightMotors.stop(hold);
}
 */
/*
void AlignRightSide(){

   const double tol = 5.0;
    const int stepMs = 10;
    const int timeoutMs = 2000;

    const double kp = 0.35;
    const int maxPct = 20;
    const double offsetMM = 54.0;

    int elapsed = 0;
    int withinTolCount = 0;
    int loopCount = 0;

    while (elapsed < timeoutMs)
    {
        double rsf = distanceMM(rightSideFront);
        double rsb = distanceMM(rightSideBack) - offsetMM;

        double err = rsf - rsb;
        double turn = 0;

        if (fabs(err) < tol)
        {
            withinTolCount++;

            if (withinTolCount >= 5)
            {
                printf("loop=%d  rsf=%.2f  rsb=%.2f  err=%.2f  turn=%.2f\n",
                       loopCount, rsf, rsb, err, turn);
                break;
            }
        }
        else
        {
            withinTolCount = 0;
            turn = kp * err;
        }

        if (turn >  maxPct) turn =  maxPct;
        if (turn < -maxPct) turn = -maxPct;

        printf("loop=%d  rsf=%.2f  rsb=%.2f  err=%.2f  turn=%.2f\n",
               loopCount, rsf, rsb, err, turn);

        leftMotors.spin(forward, turn, pct);
        rightMotors.spin(forward, -turn, pct);

        wait(stepMs, msec);
        elapsed += stepMs;
        loopCount++;
    }

    leftMotors.stop(brake);
    rightMotors.stop(brake);
}

*/

void AlignRightSide(){

    const double tol = 5.0;
    const int stepMs = 10;
    const int timeoutMs = 500;

    const double kp = 0.10; //0.35
    const double kd = 0.30;   
    const int maxPct = 20;
    const double offsetMM = 54.0;

    int elapsed = 0;
    int withinTolCount = 0;
    int loopCount = 0;

    double prevErr = 0.0;

    leftMotors.setMaxTorque(40, percent);
    rightMotors.setMaxTorque(40, percent);

    while (elapsed < timeoutMs)
    {
        double rsf = distanceMM(rightSideFront);
        double rsb = distanceMM(rightSideBack) - offsetMM;

        // Mirrored right-side error
        double err = rsb - rsf;
        double turn = 0.0;

        if (fabs(err) < tol)
        {
            withinTolCount++;

            if (withinTolCount >= 5)
            {
                printf("loop=%d  rsf=%.2f  rsb=%.2f  err=%.2f  turn=%.2f\n",
                       loopCount, rsf, rsb, err, turn);
                break;
            }
        }
        else
        {
            withinTolCount = 0;

            double derr = (err - prevErr) / (stepMs / 1000.0);

            turn = kp * err + kd * derr;
        }

        if (turn >  maxPct) turn =  maxPct;
        if (turn < -maxPct) turn = -maxPct;

        printf("loop=%d  rsf=%.2f  rsb=%.2f  err=%.2f  turn=%.2f\n",
               loopCount, rsf, rsb, err, turn);

        leftMotors.spin(forward, -turn, pct);
        rightMotors.spin(forward,  turn, pct);

        prevErr = err;

        wait(stepMs, msec);
        elapsed += stepMs;
        loopCount++;
    }

    leftMotors.stop(brake);
    rightMotors.stop(brake);
}

