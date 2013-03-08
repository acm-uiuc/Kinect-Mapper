// @author Alex Burck
// @email  burck1@illinois.edu

#ifndef ROBOT_H
#define ROBOT_H

class Robot {
	public:
		Robot();
		~Robot();
		
		int get_vel_left();
		int get_vel_right();

		void stop();
		void go_forward(int vel);
		void turn_right(int vel);
		void turn_left(int vel);
		void speed_up();
		void slow_down();
	
	private:
		float heading;
		int vl;
		int vr;
		int accel;
};

#endif
