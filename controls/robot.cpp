// @author Alex Burck
// @email  burck1@illinois.edu

#include "robot.h"

#define DEFAULT_ACCEL 4

Robot::Robot(){
	heading = 0.0;
	vl = 0;
	vr = 0;
	accel = DEFAULT_ACCEL;

	mode = MODE_PLANNER;
}

Robot::~Robot(){
}

int Robot::get_vel_left(){
	return vl;
}

int Robot::get_vel_right(){
	return vr;
}

int Robot::stop(){
	vl = vr = 0;
	return 1;
}

int Robot::go_forward(int m){
	if(mode == m){
	  //int vel = vl >= 0 ? vl : 0 - vl;
	  //vr = vl = vel > 0 ? vel : accel;
	  vl=vr=16;
		return 1;
	}
	return 0;
}

int Robot::turn_right(int m){
	if(mode == m){
		int vel = vl >= 0 ? vl : 0 - vl;
		vr = 0 - vel;
		vl = vel;
		return 1;
	}
	return 0;
}

int Robot::turn_left(int m){
	if(mode == m){
		int vel = vl >= 0 ? vl : 0 - vl;
		vr = vel;
		vl = 0 - vel;
		return 1;
	}
	return 0;
}

int Robot::speed_up(int m){
	if(mode == m){
		vr += vr >= 0 ? accel : 0 - accel;
		vl += vl >= 0 ? accel : 0 - accel;
		return 1;
	}
	return 0;
}

int Robot::slow_down(int m){
	if(mode == m){
		vr -= vr >= 0 ? accel : 0- accel;
		vl -= vl >= 0 ? accel : 0- accel;
		return 1;
	}
	return 0;
}

void Robot::set_mode(int m){
	mode = m;
}

int Robot::get_mode(){
	return mode;
}
