#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

void default_constants();

void init_cascade_position();

void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void horizontal_odom_test();
void vertical_odom_test();
void localization_test();
void tank_odom_test();
void holonomic_odom_test();
void pursuit_test();
void strong_side_2_2();