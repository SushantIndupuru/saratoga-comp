#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
  chassis.pursuit_lookahead = 10;
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){
  odom_constants();
  chassis.drive_settle_error = 0.5;
  chassis.set_coordinates(0, 0, 0);
  chassis.drive_distance(36);
  chassis.drive_stop(hold);
  while (true) {}
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  //odom_constants();
  chassis.turn_to_angle(90);
  task::sleep(1000);

  chassis.turn_to_angle(0);
  task::sleep(1000);


}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Horizontal (sideways) tracker checkout. Push the robot by hand.
 * Keep heading near 0. Slide 24" to the robot's right; X should read 24
 * and Y should stay near 0. Then spin 360 in place; X and Y should
 * stay near 0.
 */

void horizontal_odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "Horizontal odom test");
    Brain.Screen.printAt(5, 40, "Push 24in RIGHT, heading 0");
    Brain.Screen.printAt(5, 60, "X (want 24): %.2f", chassis.get_X_position());
    Brain.Screen.printAt(5, 80, "Y (want 0):  %.2f", chassis.get_Y_position());
    Brain.Screen.printAt(5, 100, "Heading:     %.2f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Sideways:    %.2f", chassis.get_SidewaysTracker_position());
    Brain.Screen.printAt(5, 140, "Forward:     %.2f", chassis.get_ForwardTracker_position());
    task::sleep(20);
  }
}

/**
 * Vertical (forward) tracker checkout. Push the robot by hand.
 * Keep heading near 0. Slide 24" forward; Y should read 24
 * and X should stay near 0.
 */

void vertical_odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "Vertical odom test");
    Brain.Screen.printAt(5, 40, "Push 24in FORWARD, heading 0");
    Brain.Screen.printAt(5, 60, "X (want 0):  %.2f", chassis.get_X_position());
    Brain.Screen.printAt(5, 80, "Y (want 24): %.2f", chassis.get_Y_position());
    Brain.Screen.printAt(5, 100, "Heading:     %.2f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Sideways:    %.2f", chassis.get_SidewaysTracker_position());
    Brain.Screen.printAt(5, 140, "Forward:     %.2f", chassis.get_ForwardTracker_position());
    task::sleep(20);
  }
}

/**
 * Full localization checkout. Push by hand; pose should match the field.
 *   1. Forward 24" along +Y  -> (0, 24, 0)
 *   2. Right 24" along +X    -> (24, 24, 0)
 *   3. Spin 360 in place     -> still (24, 24, 0)
 *   4. Push back to start    -> (0, 0, 0)
 */

void localization_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    float x = chassis.get_X_position();
    float y = chassis.get_Y_position();
    float heading = chassis.get_absolute_heading();

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "Localization test");
    Brain.Screen.printAt(5, 40, "X:       %.2f", x);
    Brain.Screen.printAt(5, 60, "Y:       %.2f", y);
    Brain.Screen.printAt(5, 80, "Heading: %.2f", heading);
    Brain.Screen.printAt(5, 100, "Fwd: %.2f  Side: %.2f", chassis.get_ForwardTracker_position(), chassis.get_SidewaysTracker_position());
    Brain.Screen.printAt(5, 140, "1. +Y 24 -> (0, 24)");
    Brain.Screen.printAt(5, 160, "2. +X 24 -> (24, 24)");
    Brain.Screen.printAt(5, 180, "3. Spin 360, pose holds");
    Brain.Screen.printAt(5, 200, "4. Back to (0, 0, 0)");

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("X:%.1f Y:%.1f", x, y);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("H:%.1f", heading);

    task::sleep(50);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Pure Pursuit checkout on a 1x2 tile strip.
 * Place the robot with its back against the near tile edge, facing
 * down the 2-tile (+Y) axis. Space is 24" wide and 48" ahead of the back.
 * Drives 36" forward, turns around, drives back, then faces 0.
 */

void pursuit_test(){
  odom_constants();
  chassis.pursuit_lookahead = 4;
  chassis.set_coordinates(0, 0, 0);

  chassis.follow_path({
    {0, 0},
    {0, 36}
  });

  chassis.turn_to_angle(180);

  chassis.follow_path({
    {0, 36},
    {0, 0}
  });

  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

void strong_side_2_2() {
  odom_constants();
  chassis.drive_settle_error = 1.5;
  Claw.set(true);
  chassis.set_coordinates(0, 0, 180);
  
  // chassis.drive_to_point(0, 4);
  // ClawDrop.set(false);
  // Roller.setVelocity(100, percent);
  // Roller.spinFor(forward, 700, degrees);
  // ClawDrop.set(true);
  // task::sleep(300);
  // chassis.drive_distance(4, 180, chassis.drive_max_voltage, chassis.heading_max_voltage, chassis.drive_settle_error, chassis.drive_settle_time, 600);
  // Claw.set(false);
  chassis.drive_distance(8, 180, 6, chassis.heading_max_voltage, chassis.drive_settle_error, chassis.drive_settle_time, 300);
  chassis.drive_distance(-5);
  task::sleep(100);
  chassis.drive_distance(10, 180, 6, chassis.heading_max_voltage, chassis.drive_settle_error, chassis.drive_settle_time, 400);
  chassis.drive_distance(-15.5);
  chassis.turn_to_angle(270);
  chassis.drive_distance(-11.75, 270, 4, chassis.heading_max_voltage, chassis.drive_settle_error, chassis.drive_settle_time, 400);
  Roller.setVelocity(5, percent);
  task::sleep(700);
  Roller.spinFor(forward, 700, degrees);
  task::sleep(700);
  chassis.drive_distance(12, 270, chassis.drive_max_voltage, chassis.heading_max_voltage, chassis.drive_settle_error, chassis.drive_settle_time, 400);
  chassis.turn_to_angle(45,12,0.5,800,9999);
  Claw.set(true);
  chassis.drive_distance(20);
  chassis.drive_max_voltage = 4;
  chassis.drive_distance(10);
  task::sleep(500);
  Claw.set(false);
  task::sleep(500);
  set_cascade_target(-100);
  task::sleep(500);
  chassis.drive_distance(8);
  chassis.turn_to_angle(180);
  chassis.drive_distance(15);

}

void weak_side() {
  odom_constants();
  chassis.drive_settle_error = 1.5;
  Claw.set(true);
  chassis.set_coordinates(0, 0, 180);

  chassis.drive_distance(8, 180, 6, chassis.heading_max_voltage, chassis.drive_settle_error, chassis.drive_settle_time, 300);
  chassis.drive_distance(-5);
  task::sleep(100);
  chassis.drive_distance(10, 180, 6, chassis.heading_max_voltage, chassis.drive_settle_error, chassis.drive_settle_time, 400);
  chassis.drive_distance(-15.5);
  chassis.turn_to_angle(90);
  chassis.drive_distance(-11.75, 90, 4, chassis.heading_max_voltage, chassis.drive_settle_error, chassis.drive_settle_time, 400);
  Roller.setVelocity(50, percent);
  task::sleep(700);
  Roller.spinFor(forward, 700, degrees);
  task::sleep(700);
  chassis.drive_distance(12, 90, chassis.drive_max_voltage, chassis.heading_max_voltage, chassis.drive_settle_error, chassis.drive_settle_time, 400);
  chassis.turn_to_angle(315,12,0.5,800,9999);
  //void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout)
  Claw.set(true);
  chassis.drive_distance(20);
  chassis.drive_max_voltage = 4;
  chassis.drive_distance(10);
  task::sleep(500);
  Claw.set(false);
  task::sleep(500);
  set_cascade_target(-100);
  task::sleep(500);
  chassis.drive_distance(8);
  chassis.turn_to_angle(180);
  chassis.drive_distance(15); //enddd

  
}