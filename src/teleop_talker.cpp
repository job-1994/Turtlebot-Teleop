// include the header file for calling the header file.

#include "teleop/teleop_talker.h"


// CONSTRUCTOR
// The method which gets called when the object is created.
// ----------------------------------------------------------------------------------------------------
//	Input	|	Type		|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	1		|	NodeHandle	|	n 			|	Links the method to the specified node, making it accessible
//

// Output	|	Type	|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	N/A 	|	N/A		|	N/A 		|	N/A

teleop_talker::teleop_talker(ros::NodeHandle &n)
	{
		//create Publisher that will publish all Twist variables to the topic cmd_vel_mux/input/teleop 
		pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
	}

// DECONSTRUCTOR
// The method which gets called when the object is stopped.
// ----------------------------------------------------------------------------------------------------
//	
//	Input	|	Type	|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	None	|	N/A  	|	N/A 		|	
//

// Output	|	Type	|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	N/A 	|	N/A		|	N/A 		|	N/A

teleop_talker::~teleop_talker(){}


// METHOD
// This method is called in the movement method, and prints out the current setspeeds of the turtlebot
// ----------------------------------------------------------------------------------------------------
//	Input	|	Type	|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	1		|	float	|	linspeed	|	stores the linear speed changed in teleop_talker::movement
// 	2		|	float	|	angspeed	|	stores the angular speed changed in teleop_talker::movement
//

// Output	|	Type	|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	None	|	N/A 	|	N/A 		|	N/A

void teleop_talker::PrintSpeed(float linspeed, float angspeed)
	{
		clear();
		teleop_talker::PrintWelcome();
		printw("\nLinear speed is now: ");
		refresh();		
		printw("%lf", linspeed);
		refresh();
		printw("\nAngular speed is now: ");
		refresh();
		printw("%lf", angspeed);
		refresh();
	}


// METHOD
// This method is called in the movement method, and prints out a welcome message and the instructions 
// for which keys to use. The printed statements appear on the ncurser window created with the initscr() method.
// ----------------------------------------------------------------------------------------------------
//	Input	|	Type	|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	None	|	N/A		|	N/A			|	N/A


// Output	|	Type	|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	None	|	void	|	N/A 		|	N/A

void teleop_talker::PrintWelcome()
{
	printw("USE NumPad TO CONTROL YOUR NEW FRIEND\n" 
			"USE I/K TO INCREMENT/DECREMENT LINEAR SPEED\n"
			"USE O/L TO INCREMENT DECREMENT ANGULAR SPEED\n"
			"USE P TO EXIT THE PROGRAM\n\n"
			"'With great power comes great responsibility.' - Job van Dieten\n"
			"'SCAR, BROOOTHER!' 'LONG LIVE THE KING!!!' - Helmi Fraser\n");
	refresh();
}


// METHOD
// This method controls the movement of the turtlebot. The keys are linked to certain velocity amounts through a
// case statement which checks which keys are pressed thorugh the ncurser library. The corresponding velocities are
// stored in the Twist message to be read by the listener class on the turtlebot minimal launch. Once they have been
// published the values are reset to 0 and new velocities are stored. The speed of the angular and linear movement 
// are stored locally and seperately. With other keys specified in the welcome message, they can be incremented and
// decremented. There are limits specified for both variables, as the speed can pose a risk for the turtlebot. 
// The linear velocity has a minimum of 0, if the variable was negative, the direction of the bot would be inversed
// The angular velocity also has a minimum, but of 0.5, as any lower velocity barely moves the bot. The incrementing
// and decrementing value can be changed in the code.
// ----------------------------------------------------------------------------------------------------------
//	Input	|	Type		|	Name		|	Description
// ----------------------------------------------------------------------------------------------------------
// 	1		|	NodeHandle	|	n 			|	Links the method to the specified node, making it accessible

// Output	|	Type		|	Name		|	Description
// ----------------------------------------------------------------------------------------------------------
// 	None	|	void		|	N/A 		|	N/A

void teleop_talker::movement(ros::NodeHandle &n)
	{
		float lin_speed = 0.4;
 		float ang_speed = 1.0;
 		float k = 0.1;

		initscr();
		teleop_talker::PrintWelcome();
		teleop_talker::PrintSpeed(lin_speed, ang_speed);
		cbreak();
		noecho();
		while (ros::ok())
			{		
				geometry_msgs::Twist cmd_stored;

				cmd_stored.linear.x = cmd_stored.angular.x = cmd_stored.linear.z 
				= cmd_stored.angular.y = cmd_stored.angular.z = cmd_stored.linear.y = 0;
	
				int ch = getch();

				switch(ch)
					{
						case 105: //i
							if(lin_speed<1.6)
								{
									lin_speed = lin_speed + k;
								 	teleop_talker::PrintSpeed(lin_speed, ang_speed);
								}
						  break;

						case 107: //k
							if(lin_speed>0.1)
								{
									lin_speed = lin_speed - k;
									teleop_talker::PrintSpeed(lin_speed, ang_speed);
								}	
 							break;

						case 111: //o
							if(ang_speed<1.6)
								{
									ang_speed = ang_speed + k;
									teleop_talker::PrintSpeed(lin_speed, ang_speed);
								}
						  break;

						case 108: //l
							if(ang_speed>0.5)
								{
									ang_speed = ang_speed - k;
									teleop_talker::PrintSpeed(lin_speed, ang_speed);
								}
							break;					
						
						case 56/*119*/: //Numpad 8 / W
							cmd_stored.linear.x = lin_speed;
							break;//move Forward

						case 52/*97*/: //Numpad 4 / A
							cmd_stored.angular.z = ang_speed; 
							break;//turn Left
						
						case 50/*115*/: //Numpad 2 / S
							cmd_stored.linear.x = -lin_speed;
							break;//move Backwards
						
						case 54/*100*/: //Numpad 6 / D
							cmd_stored.angular.z = -ang_speed;  
							break;//turn Right
						
						case 55/*113*/: //Numpad 7 / Q
							cmd_stored.linear.x = lin_speed;
							cmd_stored.angular.z = ang_speed; 
							break; //move Forward Left
						
						case 57/*101*/: //Numpad 9 / E
							cmd_stored.linear.x = lin_speed;
							cmd_stored.angular.z = -ang_speed;  
							break; //move Forward Right
						
						case 49/*122*/: //Numpad 1 / Z
							cmd_stored.linear.x = -lin_speed;
							cmd_stored.angular.z = -ang_speed;  
							break; //move Backwards Left
						
						case 51/*99*/: //Numpad 3 / C
							cmd_stored.linear.x = -lin_speed;
							cmd_stored.angular.z = ang_speed;  
							break; //move Backwards Right
						
						case 112 : //P
							endwin();
							break;
						default : 
							cmd_stored.linear.x = 0;
							cmd_stored.angular.z = 0;  
							break;	
					}			
			pub.publish(cmd_stored);
		}
	}


// MAIN METHOD
// This method executes the program. The ros platform is initiated, with the node type set to master.
// The nodehandle object is created to the variable n, and the teleop_talker node is created. The object 
// is then used to call the movement method.
// ----------------------------------------------------------------------------------------------------
//	Input	|	Type	|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	1		|	int		|	argc			|	N/A
// 	2		|	char	|	argv			|	N/A

// Output	|	Type	|	Name		|	Description
// ----------------------------------------------------------------------------------------------------
// 	None	|	void	|	N/A 		|	N/A

int main(int argc, char **argv)
	{
		ros::init(argc, argv, "Master");
		ros::NodeHandle n;
			teleop_talker tt(n);
			tt.movement(n);			
			return 0;

			
	}
