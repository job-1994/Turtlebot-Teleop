// Pragma once means the source file is only compiled once
#pragma once

//Include all relevant libraries and input/output streams
#include "ros/ros.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include "ncurses.h"


class teleop_talker
  {
// Define all private variables, which will only be available to the source file
    private:
	ros::Publisher pub; 
	float lin_speed, ang_speed, k; 
	int ch;
// Define all private variables and methods, these will be available to any class or method which creates the 
// object.
    public:
	teleop_talker(ros::NodeHandle &n);
	~teleop_talker();
	 void movement(ros::NodeHandle &n);
	 void key_numbers();
	 void PrintSpeed(float linspeed, float angspeed);
	 void modifier(char type, char sign);
	 void PrintWelcome();
};
