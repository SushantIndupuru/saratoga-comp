#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
TANK_TWO_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LeftFront, LeftBack),

//Right Motors:
motor_group(RightFront, RightBack),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT3,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.6,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT16,    -PORT20,

//LB:      //RB: 
PORT15,    -PORT19,

// Vertical / forward tracker (parallel to the chassis). Rotation sensor port:
PORT13,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
// Scaled from 1.929 after a 36" command traveled 36.75": 1.929 * 36.75/36
1.969,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

// Horizontal / sideways tracker (perpendicular to the chassis). Rotation sensor port:
PORT12,

//Sideways tracker diameter (reverse to make the direction switch):
// Scaled from 1.584 after a 24" right push read 19.21": 1.584 * 24/19.21
1.979,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

int current_auton_selection = 0;
bool auto_started = false;
bool driver_started = false;

PID cascadePID(0, 0.13, 0, 0.0, 0);
float cascade_target = 0;
bool cascade_was_manual = false;
bool cascade_b_was_pressed = false;
bool cascade_grab_raise_pending = false;
int cascade_grab_raise_ms = 0;
bool lift_was_pressed = false;
bool arm_was_pressed = false;
volatile bool cascade_async_enabled = false;

void reset_cascade_pid() {
  cascadePID.accumulated_error = 0;
  cascadePID.previous_error = 0;
}

void set_cascade_target(float target) {
  cascade_target = target;
  cascade_grab_raise_pending = false;
  reset_cascade_pid();
  cascade_async_enabled = true;
}

void init_cascade_position() {
  Cascade.resetPosition();
  Cascade.setPosition(Cascade.position(degrees) + 138, degrees);
  cascade_target = Cascade.position(degrees);
  reset_cascade_pid();
  cascade_target=0;
}

int cascade_control_task() {
  while (true) {
    if (cascade_async_enabled) {
      float output = clamp(cascadePID.compute(cascade_target - Cascade.position(degrees)), -12, 6);
      Cascade.spin(fwd, output, volt);
    }
    task::sleep(20);
  }
  return 0;
}

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  static task cascade_task(cascade_control_task);
  default_constants();

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(5, 20, "Calibrating gyro...");
  Brain.Screen.printAt(5, 40, "Keep the robot still.");
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Calibrating...");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Keep robot still");

  chassis.Gyro.calibrate();
  while (chassis.Gyro.isCalibrating()) {
    task::sleep(20);
  }

  Brain.Screen.printAt(5, 60, "Gyro ready.");
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Gyro ready");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Heading: %.1f", chassis.get_absolute_heading());
  Controller1.rumble(".");
  task::sleep(500);

  while(!auto_started && !driver_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "Strong Side");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "Weak side");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "Auton 2");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "Horizontal Odom");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "Localization");
        break;
      case 5:
        Brain.Screen.printAt(5, 140, "Vertical Odom");
        break;
      case 6:
        Brain.Screen.printAt(5, 140, "Auton 6");
        break;
      case 7:
        Brain.Screen.printAt(5, 140, "Pursuit Test");
        break;
    }
    Brain.Screen.printAt(5, 180, "Cascade: %.1f deg", Cascade.position(degrees));
    Brain.Screen.printAt(5, 200, "Claw Dist: %.1f mm", ClawDistance.objectDistance(mm));
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  init_cascade_position();
  switch(current_auton_selection){ 
    case 0:
      strong_side_2_2();
      break;
    case 1:         
      weak_side();
      break;
    case 2:
      turn_test();
      break;
    case 3:
      horizontal_odom_test();
      break;
    case 4:
      localization_test();
      break;
    case 5:
      vertical_odom_test();
      break;
    case 6:
      tank_odom_test();
      break;
    case 7:
      pursuit_test();
      break;
 }
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
  cascade_async_enabled = false;

  if (!auto_started) {
    init_cascade_position();
  }

  driver_started = true;
  Brain.Screen.clearScreen();

  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //Replace this line with chassis.control_tank(); for tank drive 
    //or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();

    if(Controller1.ButtonY.pressing()) {
      Roller.spin(reverse, 12, volt);
    } else {
      Roller.stop(hold);
    }

    if (Controller1.ButtonR2.pressing()) {
      Cascade.spin(reverse, 12, volt);
      cascade_target = Cascade.position(degrees);
      cascade_was_manual = true;
    } else if (Controller1.ButtonL2.pressing()) {
      Cascade.spin(fwd, 6, volt);
      cascade_target = Cascade.position(degrees);
      cascade_was_manual = true;
    } else {
      if (cascade_was_manual) {
        cascade_target = Cascade.position(degrees);
        reset_cascade_pid();
        cascade_was_manual = false;
      }
      if (Controller1.ButtonB.pressing()) {
        if (!cascade_b_was_pressed) {
          cascade_target = 0;
          reset_cascade_pid();
        }
        cascade_b_was_pressed = true;
      } else {
        cascade_b_was_pressed = false;
      }
      float output = clamp(cascadePID.compute(cascade_target - Cascade.position(degrees)), -12, 6);
      Cascade.spin(fwd, output, volt);
    }

    if (Controller1.ButtonL1.pressing()) {
      Claw.set(true);
      cascade_grab_raise_pending = false;
    } else if (ClawDistance.objectDistance(mm) < -100) { //never trigger
      Claw.set(false);
      if (cascade_target == 0 && !cascade_was_manual && !cascade_grab_raise_pending) {
        cascade_grab_raise_pending = true;
        cascade_grab_raise_ms = Brain.Timer.time(msec);
      }
    } else {
      Claw.set(Controller1.ButtonR1.pressing());
    }

    if (Controller1.ButtonRight.pressing()) {
      if (!lift_was_pressed) {
        RightLift.set(!RightLift.value());
        ClawDrop.set(!RightLift.value());
        
      }
      lift_was_pressed = true;
    } else {
      lift_was_pressed = false;
    }

    if (Controller1.ButtonDown.pressing()) {
      if (!arm_was_pressed) {
        RightArm.set(!RightArm.value());
      }
      arm_was_pressed = true;
    } else {
      arm_was_pressed = false;
    }

    if (cascade_was_manual) {
      cascade_grab_raise_pending = false;
    }

    if (cascade_grab_raise_pending && Brain.Timer.time(msec) - cascade_grab_raise_ms >= 100) {
      cascade_target -= 200;
      reset_cascade_pid();
      cascade_grab_raise_pending = false;
    }

    // Brain.Screen.clearScreen();
    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.print("Cascade: %.1f deg", Cascade.position(degrees));
    // Brain.Screen.setCursor(2, 1);
    // Brain.Screen.print("Claw Dist: %.1f mm", ClawDistance.objectDistance(mm));
    // Controller1.Screen.clearScreen();
    // Controller1.Screen.setCursor(1, 1);
    // Controller1.Screen.print("Cas: %.1f deg", Cascade.position(degrees));
    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.print("Claw: %.1f mm", ClawDistance.objectDistance(mm));

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
