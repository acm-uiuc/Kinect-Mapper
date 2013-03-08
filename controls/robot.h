// @author Alex Burck
// @email  burck1@illinois.edu

#ifndef ROBOT_H
#define ROBOT_H

#define MODE_MANUAL 0
#define MODE_PLANNER 1

class Robot {
	public:
		Robot();
		~Robot();
		
		int get_vel_left();
		int get_vel_right();

		int stop();
		int go_forward(int m);
		int turn_right(int m);
		int turn_left(int m);
		int speed_up(int m);
		int slow_down(int m);

		void set_mode(int mode);
		int get_mode();
	
	private:
		float heading;
		int vl;
		int vr;
		int accel;
		int mode;
};

#endif
