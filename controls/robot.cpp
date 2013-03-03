// @author Alex Burck
// @email  burck1@illinois.edu

#include "quadtree.h"

#define DEFAULT_ACCEL 4

Robot::Robot(){
	heading = 0.0;
	vl = 0;
	vr = 0;
	accel = DEFAULT_ACCEL;
}

int Robot::get_vel_left(){
	return vl;
}

int Robot::get_vel_right(){
	return vr;
}

void Robot::stop(){
	vl = vr = 0;
}

void Robot::go_forward(int vel){
	vr = vl = vel > 0 ? vel : accel;
}

void Robot::turn_right(int vel){
	vr = 0 - vel;
	vl = vel;
}

void Robot::turn_left(int vel){
	vr = vel;
	vl = 0 - vel;
}

void Robot::speed_up(){
	vr += vr >= 0 ? accel : 0 - accel;
	vl += vl >= 0 ? accel : 0 - accel;
}

void Robot::slow_down(){
	vr -= vr >= 0 ? accel : 0- accel;
	vl -= vl >= 0 ? accel : 0- accel;
}
