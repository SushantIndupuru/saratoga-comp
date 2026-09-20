#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

// Drive
motor LeftFront = motor(PORT16, ratio6_1, true);
motor LeftBack = motor(PORT15, ratio6_1, true);
motor RightFront = motor(PORT21, ratio6_1, false);
motor RightBack = motor(PORT19, ratio6_1, false);

// Mechanisms
motor Roller = motor(PORT2, ratio6_1, false);
motor Cascade1 = motor(PORT14, ratio18_1, false);
motor Cascade2 = motor(PORT18, ratio18_1, true);
motor Cascade3 = motor(PORT17, ratio18_1, false);
motor_group Cascade = motor_group(Cascade1, Cascade2, Cascade3);

// Sensors and pneumatics (IMU and odom rotation sensors are owned by the Drive chassis)
digital_out Claw = digital_out(Brain.ThreeWirePort.A);
digital_out ClawDrop = digital_out(Brain.ThreeWirePort.H);
digital_out RightLift = digital_out(Brain.ThreeWirePort.C);
digital_out RightArm = digital_out(Brain.ThreeWirePort.B);
distance ClawDistance = distance(PORT10);
rotation RollerRotation = rotation(PORT9);

controller Controller1 = controller(primary);

void vexcodeInit( void ) {
  Cascade.setStopping(hold);
}
