#include "vex.h"

/**
 * Drive constructor for the chassis.
 * Even though there's only one constructor, there can be
 * huge differences in implementation depending on the drive style
 * selected.
 * 
 * @param drive_setup The style of drive, such as TANK_TWO_ROTATION.
 * @param DriveL Left motor group.
 * @param DriveR Right motor group.
 * @param gyro_port IMU port.
 * @param wheel_diameter Wheel diameter in inches.
 * @param wheel_ratio External drive gear ratio.
 * @param gyro_scale Scale factor in degrees.
 * @param DriveLF_port Left front port for holonomic drives.
 * @param DriveRF_port Right front port for holonomic drives.
 * @param DriveLB_port Left back port for holonomic drives.
 * @param DriveRB_port Right back port for holonomic drives.
 * @param ForwardTracker_port Port for the forward tracker.
 * @param ForwardTracker_diameter Diameter in inches.
 * @param ForwardTracker_center_distance Horizontal distance in inches.
 * @param SidewaysTracker_port Port for the sideways tracker.
 * @param SidewaysTracker_diameter Diameter in inches.
 * @param SidewaysTracker_center_distance Vertical distance in inches.
 */

Drive::Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, 
int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, 
int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, 
int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, 
int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance) :
  wheel_diameter(wheel_diameter),
  wheel_ratio(wheel_ratio),
  gyro_scale(gyro_scale),
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0),
  drive_setup(drive_setup),
  DriveL(DriveL),
  DriveR(DriveR),
  Gyro(inertial(gyro_port)),
  DriveLF(abs(DriveLF_port), is_reversed(DriveLF_port)),
  DriveRF(abs(DriveRF_port), is_reversed(DriveRF_port)),
  DriveLB(abs(DriveLB_port), is_reversed(DriveLB_port)),
  DriveRB(abs(DriveRB_port), is_reversed(DriveRB_port)),
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  E_ForwardTracker(ThreeWire.Port[to_port(ForwardTracker_port)]),
  E_SidewaysTracker(ThreeWire.Port[to_port(SidewaysTracker_port)])
{
    if (drive_setup == TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_ONE_FORWARD_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
      odom.set_physical_distances(ForwardTracker_center_distance, 0);
    } 
    if (drive_setup == TANK_ONE_SIDEWAYS_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ROTATION || 
    drive_setup == TANK_TWO_ENCODER || drive_setup == TANK_TWO_ROTATION ||
    drive_setup == HOLONOMIC_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ROTATION){
      odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    }
}

/**
 * Drives each side of the chassis at the specified voltage.
 * 
 * @param leftVoltage Voltage out of 12.
 * @param rightVoltage Voltage out of 12.
 */

void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
  DriveL.spin(fwd, leftVoltage, volt);
  DriveR.spin(fwd, rightVoltage,volt);
}

/**
 * Resets default turn constants.
 * Turning includes turn_to_angle() and turn_to_point().
 * 
 * @param turn_max_voltage Max voltage out of 12.
 * @param turn_kp Proportional constant.
 * @param turn_ki Integral constant.
 * @param turn_kd Derivative constant.
 * @param turn_starti Minimum angle in degrees for integral to begin.
 */

void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

void Drive::set_turn_curve(float turn_curve){
  this->turn_curve = turn_curve;
}

/**
 * Resets default drive constants.
 * Driving includes drive_distance(), drive_to_point(), and
 * holonomic_drive_to_point().
 * 
 * @param drive_max_voltage Max voltage out of 12.
 * @param drive_kp Proportional constant.
 * @param drive_ki Integral constant.
 * @param drive_kd Derivative constant.
 * @param drive_starti Minimum distance in inches for integral to begin
 */

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

/**
 * Resets default heading constants.
 * Heading control keeps the robot facing the right direction
 * and is part of drive_distance(), drive_to_point(), and
 * holonomic_drive_to_point.
 * 
 * @param heading_max_voltage Max voltage out of 12.
 * @param heading_kp Proportional constant.
 * @param heading_ki Integral constant.
 * @param heading_kd Derivative constant.
 * @param heading_starti Minimum angle in degrees for integral to begin.
 */

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

/**
 * Resets default swing constants.
 * Swing control holds one side of the drive still and turns with the other.
 * Only left_swing_to_angle() and right_swing_to_angle() use these constants.
 * 
 * @param swing_max_voltage Max voltage out of 12.
 * @param swing_kp Proportional constant.
 * @param swing_ki Integral constant.
 * @param swing_kd Derivative constant.
 * @param swing_starti Minimum angle in degrees for integral to begin.
 */

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

/**
 * Resets default turn exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param turn_settle_error Error to be considered settled in degrees.
 * @param turn_settle_time Time to be considered settled in milliseconds.
 * @param turn_timeout Time before quitting and move on in milliseconds.
 */

void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

/**
 * Resets default drive exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param drive_settle_error Error to be considered settled in inches.
 * @param drive_settle_time Time to be considered settled in milliseconds.
 * @param drive_timeout Time before quitting and move on in milliseconds.
 */

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

/**
 * Resets default swing exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param swing_settle_error Error to be considered settled in degrees.
 * @param swing_settle_time Time to be considered settled in milliseconds.
 * @param swing_timeout Time before quitting and move on in milliseconds.
 */

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

/**
 * Gives the drive's absolute heading with Gyro correction.
 * 
 * @return Gyro scale-corrected heading in the range [0, 360).
 */

float Drive::get_absolute_heading(){ 
  return( reduce_0_to_360( Gyro.rotation()*360.0/gyro_scale ) ); 
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Left position in inches.
 */

float Drive::get_left_position_in(){
  return( DriveL.position(deg)*drive_in_to_deg_ratio );
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Right position in inches.
 */

float Drive::get_right_position_in(){
  return( DriveR.position(deg)*drive_in_to_deg_ratio );
}

/**
 * Stops both sides of the drive with the desired mode.
 * 
 * @param mode hold, brake, or stop
 */

void Drive::drive_stop(vex::brakeType mode){
  DriveL.stop(mode);
  DriveR.stop(mode);
}

/**
 * Turns the robot to a field-centric angle.
 * Optimizes direction, so it turns whichever way is closer to the 
 * current heading of the robot.
 * 
 * @param angle Desired angle in degrees.
 */

void Drive::turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while( !turnPID.is_settled() ){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
}

/**
 * Drives the robot a given distance with a given heading.
 * Drive distance does not optimize for direction, so it won't try
 * to drive at the opposite heading from the one given to get there faster.
 * You can control the heading, but if you choose not to, it will drive with the
 * heading it's currently facing. Distance is measured from odometry (tracking
 * wheels), not the drive motors.
 *
 * @param distance Desired distance in inches.
 * @param heading Desired heading in degrees.
 */

void Drive::drive_distance(float distance){
  drive_distance(distance, get_absolute_heading(), drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  if (!odom_started){
    start_odom(get_X_position(), get_Y_position(), get_absolute_heading());
  }

  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_x = get_X_position();
  float start_y = get_Y_position();
  float heading_rad = to_rad(heading);
  while(drivePID.is_settled() == false){
    float traveled = (get_X_position() - start_x) * sin(heading_rad) + (get_Y_position() - start_y) * cos(heading_rad);
    float drive_error = distance - traveled;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
}

/**
 * Turns to a given angle with only one side of the drivetrain.
 * Like turn_to_angle(), is optimized for turning the shorter
 * direction.
 * 
 * @param angle Desired angle in degrees.
 */

void Drive::left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveL.spin(fwd, output, volt);
    DriveR.stop(hold);
    task::sleep(10);
  }
}

void Drive::right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveR.spin(reverse, output, volt);
    DriveL.stop(hold);
    task::sleep(10);
  }
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::get_ForwardTracker_position(){
  if (drive_setup==ZERO_TRACKER_ODOM || drive_setup == TANK_ONE_SIDEWAYS_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ROTATION){
    return(get_right_position_in());
  }
  if (drive_setup==TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }else{
    return(R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::get_SidewaysTracker_position(){
  if (drive_setup==TANK_ONE_FORWARD_ENCODER || drive_setup == TANK_ONE_FORWARD_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
    return(0);
  }else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER || drive_setup == TANK_ONE_SIDEWAYS_ENCODER){
    return(E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }else{
    return(R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }
}

/**
 * Background task for updating the odometry.
 */

void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    task::sleep(5);
  }
}

/**
 * Resets the robot's heading.
 * For example, at the beginning of auton, if your robot starts at
 * 45 degrees, so set_heading(45) and the robot will know which way 
 * it's facing.
 * 
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_heading(float orientation_deg){
  Gyro.setRotation(orientation_deg*gyro_scale/360.0, deg);
}

/**
 * Resets the robot's coordinates and heading.
 * This is for odom-using robots to specify where the bot is at the beginning
 * of the match.
 * 
 * @param X_position Robot's x in inches.
 * @param Y_position Robot's y in inches.
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  start_odom(X_position, Y_position, orientation_deg);
}

void Drive::start_odom(float X_position, float Y_position, float orientation_deg){
  R_ForwardTracker.resetPosition();
  R_SidewaysTracker.resetPosition();
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  set_heading(orientation_deg);
  if (!odom_started){
    odom_task = task(position_track_task);
    odom_started = true;
  }
}

/**
 * Gets the robot's x.
 * 
 * @return The robot's x position in inches.
 */

float Drive::get_X_position(){
  return(odom.X_position);
}

/**
 * Gets the robot's y.
 * 
 * @return The robot's y position in inches.
 */

float Drive::get_Y_position(){
  return(odom.Y_position);
}

/**
 * Drives to a specified point on the field.
 * Uses the double-PID method, with one for driving and one for heading correction.
 * The drive error is the euclidean distance to the desired point, and the heading error
 * is the turn correction from the current heading to the desired point. Uses optimizations
 * like driving backwards whenever possible and scaling the drive output with the cosine
 * of the angle to the point.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 */

void Drive::drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  float start_angle_deg = to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()));
  PID headingPID(start_angle_deg-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = false;
  bool prev_line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
  while(!drivePID.is_settled()){
    line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    task::sleep(10);
  }
}

/**
 * Drives to a specified point and orientation on the field.
 * Uses a boomerang controller. The carrot point is back from the target
 * by the same distance as the robot's distance to the target, times the lead. The
 * robot always tries to go to the carrot, which is constantly moving, and the
 * robot eventually gets into position. The heading correction is optimized to only
 * try to reach the correct angle when drive error is low, and the robot will drive 
 * backwards to reach a pose if it's faster. .5 is a reasonable value for the lead. 
 * The setback parameter is used to glide into position more effectively. It is
 * the distance back from the target that the robot tries to drive to first.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param angle Desired orientation in degrees.
 * @param lead Constant scale factor that determines how far away the carrot point is. 
 * @param setback Distance in inches from target by which the carrot is always pushed back.
 * @param drive_min_voltage Minimum voltage on the drive, used for chaining movements.
 */

void Drive::drive_to_pose(float X_position, float Y_position, float angle){
  drive_to_pose(X_position, Y_position, angle, boomerang_lead, boomerang_setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}


void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  float target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());
  PID drivePID(target_distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool prev_line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool crossed_center_line = false;
  bool center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
  bool prev_center_line_side = center_line_side;
  while(!drivePID.is_settled()){
    line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
    if(center_line_side != prev_center_line_side){
      crossed_center_line = true;
    }

    target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());

    float carrot_X = X_position - sin(to_rad(angle)) * (lead * target_distance + setback);
    float carrot_Y = Y_position - cos(to_rad(angle)) * (lead * target_distance + setback);

    float drive_error = hypot(carrot_X-get_X_position(),carrot_Y-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(carrot_X-get_X_position(),carrot_Y-get_Y_position()))-get_absolute_heading());

    if (drive_error<drive_settle_error || crossed_center_line || drive_error < setback) { 
      heading_error = reduce_negative_180_to_180(angle-get_absolute_heading()); 
      drive_error = target_distance;
    }
    
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    task::sleep(10);
  }
}

/**
 * Turns to a specified point on the field.
 * Functions similarly to turn_to_angle() except with a point. The
 * extra_angle_deg parameter turns the robot extra relative to the 
 * desired target. For example, if you want the back of your robot
 * to point at (36, 42), you would run turn_to_point(36, 42, 180).
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param extra_angle_deg Angle turned past the desired heading in degrees.
 */

void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
}

/**
 * Drives and turns simultaneously to a desired pose.
 * Uses two PID loops, one drive and one heading to drive and turn
 * at the same time. Optimized to turn the quicker direction and only
 * exits once both PID loops have settled. It uses the heading constants
 * for heading but the turn exit conditions to settle.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param angle Desired ending angle in degrees.
 */

void Drive::holonomic_drive_to_pose(float X_position, float Y_position){
  holonomic_drive_to_pose(X_position, Y_position, get_absolute_heading(), drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle){
  holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage){
  holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  holonomic_drive_to_pose(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_pose(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID turnPID(angle-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while( !(drivePID.is_settled() && turnPID.is_settled()) ){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float turn_error = reduce_negative_180_to_180(angle-get_absolute_heading());

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    turn_output = clamp(turn_output, -heading_max_voltage, heading_max_voltage);

    float heading_error = atan2(Y_position-get_Y_position(), X_position-get_X_position());

    DriveLF.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) + turn_output, volt);
    DriveLB.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) + turn_output, volt);
    DriveRB.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) - turn_output, volt);
    DriveRF.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) - turn_output, volt);
    task::sleep(10);
  }
}

/**
 * Follows a polyline with Pure Pursuit.
 * Each cycle finds a lookahead point on the path and uses the same
 * drive/heading PID pair as drive_to_point() to steer toward it.
 * Speed is based on remaining path length so the robot decelerates
 * into the last waypoint. Progress only moves forward along the path.
 *
 * @param path Waypoints in inches, field-centric. 0 degrees is +Y.
 * @param lookahead Lookahead radius in inches. Larger is smoother and
 * cuts corners more; smaller tracks the path more tightly.
 * @param reverse If true, follows the path driving backward.
 */

void Drive::follow_path(const std::vector<Point> &path){
  follow_path(path, pursuit_lookahead, false, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow_path(const std::vector<Point> &path, float lookahead){
  follow_path(path, lookahead, false, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow_path(const std::vector<Point> &path, float lookahead, bool reverse){
  follow_path(path, lookahead, reverse, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow_path(const std::vector<Point> &path, float lookahead, bool reverse, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  follow_path(path, lookahead, reverse, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow_path(const std::vector<Point> &path, float lookahead, bool reverse, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  follow_path(path, lookahead, reverse, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::follow_path(const std::vector<Point> &path, float lookahead, bool reverse, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  if (path.empty()){
    return;
  }
  if (!odom_started){
    start_odom(get_X_position(), get_Y_position(), get_absolute_heading());
  }
  if (path.size() == 1){
    drive_to_point(path[0].x, path[0].y, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
    return;
  }

  PurePursuit pursuit;
  pursuit.set_path(path);

  Point end = path.back();
  float end_angle = to_deg(atan2(end.x - path[path.size() - 2].x, end.y - path[path.size() - 2].y));
  if (reverse){
    end_angle = reduce_0_to_360(end_angle + 180);
  }

  float remaining = pursuit.get_remaining_distance(get_X_position(), get_Y_position());
  float end_error = hypot(end.x - get_X_position(), end.y - get_Y_position());
  PID drivePID(fmax(remaining, end_error), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(0, heading_kp, heading_ki, heading_kd, heading_starti);

  bool line_settled = false;
  bool prev_line_settled = is_line_settled(end.x, end.y, end_angle, get_X_position(), get_Y_position());
  while (!drivePID.is_settled()){
    float robot_x = get_X_position();
    float robot_y = get_Y_position();
    remaining = pursuit.get_remaining_distance(robot_x, robot_y);
    end_error = hypot(end.x - robot_x, end.y - robot_y);

    line_settled = is_line_settled(end.x, end.y, end_angle, robot_x, robot_y);
    if (line_settled && !prev_line_settled && end_error < lookahead){
      break;
    }
    prev_line_settled = line_settled;

    Point look = pursuit.get_lookahead_point(robot_x, robot_y, lookahead);
    float look_dist = hypot(look.x - robot_x, look.y - robot_y);
    float drive_error = fmax(remaining, end_error);

    float heading_error;
    if (look_dist < 0.5){
      heading_error = reduce_negative_180_to_180(end_angle - get_absolute_heading());
    } else {
      heading_error = reduce_negative_180_to_180(to_deg(atan2(look.x - robot_x, look.y - robot_y)) - get_absolute_heading());
    }

    bool going_reverse = reverse;
    if (reverse){
      heading_error = reduce_negative_180_to_180(heading_error + 180);
    } else if (fabs(heading_error) > 90){
      going_reverse = true;
      heading_error = reduce_negative_180_to_180(heading_error + 180);
    }

    float drive_output = drivePID.compute(drive_error);
    if (going_reverse){
      drive_output = -drive_output;
    }

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output *= heading_scale_factor;
    float heading_output = headingPID.compute(heading_error);

    if (drive_error < drive_settle_error){
      heading_output = 0;
    }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor) * drive_max_voltage, fabs(heading_scale_factor) * drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    if (going_reverse){
      heading_output = -heading_output;
    }

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    task::sleep(10);
  }
}

/**
 * Controls a chassis with left stick throttle and right stick turning.
 * Default deadband is 5.
 */

void Drive::control_arcade(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  turn = copysign(pow(fabs(turn / 100.0), turn_curve) * 100.0, turn);
  DriveL.spin(fwd, to_volt(throttle+turn), volt);
  DriveR.spin(fwd, to_volt(throttle-turn), volt);
}

/**
 * Controls a chassis with left stick throttle and strafe, and right stick turning.
 * Default deadband is 5.
 */

void Drive::control_holonomic(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  turn = copysign(pow(fabs(turn / 100.0), turn_curve) * 100.0, turn);
  float strafe = deadband(controller(primary).Axis4.value(), 5);
  DriveLF.spin(fwd, to_volt(throttle+turn+strafe), volt);
  DriveRF.spin(fwd, to_volt(throttle-turn-strafe), volt);
  DriveLB.spin(fwd, to_volt(throttle+turn-strafe), volt);
  DriveRB.spin(fwd, to_volt(throttle-turn+strafe), volt);
}

/**
 * Controls a chassis with left stick left drive and right stick right drive.
 * Default deadband is 5.
 */

void Drive::control_tank(){
  float leftthrottle = deadband(controller(primary).Axis3.value(), 5);
  float rightthrottle = deadband(controller(primary).Axis2.value(), 5);
  DriveL.spin(fwd, to_volt(leftthrottle), volt);
  DriveR.spin(fwd, to_volt(rightthrottle), volt);
}

/**
 * Tracking task to run in the background.
 */

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}