#include "vex.h"

using namespace vex;

extern controller controller1;

extern brain robotBrain;  

//drivetrain motors
extern motor left1Mtr;
extern motor left2Mtr;
extern motor left3Mtr;
extern motor left4Mtr;
extern motor_group leftMotors;
extern motor right1Mtr;
extern motor right2Mtr;
extern motor right3Mtr;
extern motor right4Mtr;
extern motor_group rightMotors;

extern motor leftStrafeMotor;
extern motor rightStrafeMotor;
extern motor_group strafeMotors;


//Objective motor constuctors
extern motor conv;

extern motor frontIntake;
extern motor centerIntake;
extern motor upperIntake;
extern motor topIntake;
extern motor_group intakeMotors;
extern motor_group noTopIntake;

extern motor descoreAll;

//Pnuematic constructors
extern digital_out unknownHammer;
extern digital_out blockingWizard;

extern triport wireExpander;
extern digital_out descore;

//Sensor constructors
extern inertial inertialSensor;
extern gps leftGpsSensor;
extern gps rightGpsSensor;
extern rotation rightEncoder;
extern rotation leftEncoder;

extern vex::analog_in leftSideFront;
extern vex::analog_in leftSideBack;
extern vex::analog_in rightSideFront;
extern vex::analog_in rightSideBack;
extern vex::analog_in leftBack;
extern vex::analog_in rightBack;


extern aivision::colordesc redObj;
extern aivision::colordesc blueObj;
extern aivision::colordesc greenObj;
extern aivision::colordesc whiteObj;
extern aivision::colordesc grayObj;
extern aivision aiVisionE;

extern double turnKp;
extern double turnKd;
extern double turnKi;
extern double straightKp;  
extern double straightKd;


////////////////////////////////////////////////////////////////////
//FUNCTION EXTERNS-------------------------------------------------
///////////////////////////////////////////////////////////////////


extern void driveFoward(int speed, int time);
extern void driveReverse(int speed, int time);
extern void driveFowardWithHeading(double speed, double time, double kp, double kd);


extern void maintainConstantY_DirectionPositiveXRight(double kp, double kd, double desiredY, double desiredX, double speed);
extern void maintainConstantY_DirectionNegativeXRight(double kp, double kd, double desiredY, double desiredX, double speed);
extern void maintainConstantX_DirectionPositiveYRight(double kp, double kd, double desiredX, double desiredY, double speed);
extern void maintainConstantX_DirectionNegativeYRight(double kp, double kd, double desiredX, double desiredY, double speed);

extern void maintainConstantY_DirectionPositiveXLeft(double kp, double kd, double desiredY, double desiredX, double speed);
extern void maintainConstantY_DirectionNegativeXLeft(double kp, double kd, double desiredY, double desiredX, double speed);
extern void maintainConstantX_DirectionPositiveYLeft(double kp, double kd, double desiredX, double desiredY, double speed);
extern void maintainConstantX_DirectionNegativeYLeft(double kp, double kd, double desiredX, double desiredY, double speed);

extern void rotateInertialCounterClockWisePID(double angle, double kp, double kd, double ki);
extern void rotateInertialClockWisePID(double angle, double kp, double kd, double ki);

extern void forwardStraightToPositiveXPosition(double xPosition);
extern void forwardStraightToNegativeXPosition(double xPosition);

extern void trackToCenterRedLeft(double kp, double kd);
extern void trackToCenterBlueLeft(double kp, double kd, double offset);

extern void ramAndIntakeSkills(int cycles);

extern void unknownHammerUp();
extern void unknownHammerDown();
extern void scoreBlock();
extern void scoreAllow();

extern void cookieMonsterOne();

extern void driveLeftWithH(double speed, double time);
extern void scoringTopSkills(double speed, double time);
extern void scoringTopAuton(double speed, double smallTime, double time);

extern void testDist();
extern double distanceMM(vex::analog_in& sensor);
extern void strafeWithDistanceL(float desiredD, float kp);
extern void strafeWithDistanceR(float desiredD, float kp);
extern void AlignLeftSide();
extern void AlignRightSide();

extern void rightTurnAlign();
extern void leftTurnAlign();
extern void parkingTurn();

extern void alignWithDistanceR(float desiredD, double kp, int speed);
extern void alignWithDistanceL(float desiredD, double kp);

extern void openDescore();
extern void closeDescore();