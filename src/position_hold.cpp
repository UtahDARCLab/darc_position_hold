// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

#define THRUST  0
#define ROLL    1
#define PITCH   2
#define YAW     3

// u_out is the output of the node as new_u
// u_curr is the current input, possibly from joystick or something
geometry_msgs::Twist u_out, u_curr;

// curr_pos is the position input from mocap
// curr_vel is velocity input from mocap, found from first-order derivative
// curr_yaw is global yaw from mocap
// curr_yaw_vel is global yaw velocity from mocap, also first-order derivative
geometry_msgs::Vector3 curr_pos, curr_vel;
geometry_msgs::Vector3 desired_position, offset_position;
double curr_yaw, curr_yaw_vel;

// right bumper for turning on PID on position
// left bumper for setting the "zero" position of robot
// xbox button for cancelling relative positioning of robot
double right_button, reset_button, offset_button;

// proportional, integral, and derivative gain arrays
double Kp[4], Ki[4], Kd[4];

// proportional, integral, and derivative control effort arrays
double propErr[4], intErr[4], derivErr[4];

// control effort out for PID on each axis
double controlEffort[4];

// Read xbox buttons
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	right_button = joy_msg_in.buttons[5];
	reset_button = joy_msg_in.buttons[8];
	offset_button = joy_msg_in.buttons[4];
}

// Read mocap position, allow for relative positioning with left bumper and reset button
void pos_callback(const geometry_msgs::Vector3& pos_msg_in)
{
    if(reset_button)
    {
        offset_position.x = 0.0;
        offset_position.y = 0.0;
        offset_position.z = 0.0;
    }
    else if(offset_button){
        offset_position.x = pos_msg_in.x;
        offset_position.y = pos_msg_in.y;
        offset_position.z = pos_msg_in.z;
    }
	curr_pos.x = pos_msg_in.x - offset_position.x;
	curr_pos.y = pos_msg_in.y - offset_position.y;
	curr_pos.z = pos_msg_in.z - offset_position.z;
}

// Read mocap yaw angle
void yaw_callback(const std_msgs::Float32& yaw_msg_in)
{
	curr_yaw = yaw_msg_in.data;
}

// Read mocap yaw angular velocity
void yawVel_callback(const geometry_msgs::Vector3& rdot_msg_in)
{
	curr_yaw_vel = rdot_msg_in.z;
}

// Read mocap velocity (first-order derivative)
void vel_callback(const geometry_msgs::Vector3& vel_msg_in)
{
	curr_vel.x = vel_msg_in.x;
	curr_vel.y = vel_msg_in.y;
	curr_vel.z = vel_msg_in.z;
}

// Read current input, typically from joystick 
void u_callback(const geometry_msgs::Twist& u_msg_in)
{
    u_curr.angular.x = u_msg_in.angular.x;
    u_curr.angular.y = u_msg_in.angular.y;
    u_curr.angular.z = u_msg_in.angular.z;
    u_curr.linear.z  = u_msg_in.linear.z;
}

// Read desired position, typically from some waypoint node
void desired_position_callback(const geometry_msgs::Vector3& des_pos_msg_in)
{
    desired_position.x = des_pos_msg_in.x;
    desired_position.y = des_pos_msg_in.y;
    desired_position.z = des_pos_msg_in.z;
}

// function to calculate control effort on a given axis
void calculateControlEffort(int index, double desPoint, double currPoint, double currVel);

// function to check for integral windup on a given axis
void checkForWindup(int index, double& control, double desPoint, double currPoint);

int main(int argc, char** argv)
{
    // ROS Initialization stuff
    ros::init(argc,argv,"position_hold");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    // ROS publishers: output new_u to apply to robot
    ros::Publisher u_pub;
    u_pub = node.advertise<geometry_msgs::Twist>("new_u",1);

    // ROS subscribers: joy directly and desired_u (mapped joysticks on controller)
    ros::Subscriber joy_sub, u_sub;
    joy_sub = node.subscribe("joy",1,joy_callback);
    u_sub = node.subscribe("desired_u",1,u_callback);
    
    // Mocap ROS subscribers
    ros::Subscriber pos_sub, vel_sub, yaw_sub, yaw_vel_sub;
    pos_sub = node.subscribe("current_position",1,pos_callback);
    vel_sub = node.subscribe("current_velocity",1,vel_callback);
    yaw_sub = node.subscribe("current_yaw",1,yaw_callback);
    yaw_vel_sub = node.subscribe("current_rdot",1,yawVel_callback);
    
    // waypoint ROS subscribers
    ros::Subscriber desired_pos_sub;
    desired_pos_sub = node.subscribe("desired_position",1,desired_position_callback);
    
    // Thrust proportional, integral, and derivative gains from launch file
    node.getParam("/thrustP",Kp[THRUST]);
    node.getParam("/thrustI",Ki[THRUST]);
    node.getParam("/thrustD",Kd[THRUST]);
    node.getParam("/rollP",  Kp[ROLL]);
    node.getParam("/rollI",  Ki[ROLL]);
    node.getParam("/rollD",  Kd[ROLL]);
    node.getParam("/pitchP", Kp[PITCH]);
    node.getParam("/pitchI", Ki[PITCH]);
    node.getParam("/pitchD", Kd[PITCH]);
    node.getParam("/yawP",   Kp[YAW]);
    node.getParam("/yawI",   Ki[YAW]);
    node.getParam("/yawD",   Kd[YAW]);

    bool flag = false;
    // Loop until node closed or some ROS crash/error
    while(ros::ok())
    {
        // Read in subscribed messages
        ros::spinOnce();
                
        //seding out to make sure it reset
        // Send every other loop to slow down scree print
        flag = !flag;
        if(flag)
        {
            ROS_INFO("new_position [%.4f, %.4f, %.4f]",curr_pos.x, curr_pos.y, curr_pos.z);
        }

        if(!right_button) // if button not pressed, reset integral terms to prevent windup
        {
            for(int i = 0; i < 4; i++)
            {
                intErr[i] = 0.0;
            }
        }
        
        // Calculate PID control effort for each axis
        calculateControlEffort(THRUST,  desired_position.z,  curr_pos.z,  curr_vel.z);
        calculateControlEffort(ROLL,    desired_position.y,  curr_pos.y,  curr_vel.y);
        calculateControlEffort(PITCH,   desired_position.x,  curr_pos.x,  curr_vel.x);
        calculateControlEffort(YAW,     0.0,                 curr_yaw,    curr_yaw_vel);
        
        // Apply thrust, roll, pitch, and yaw control
        u_out.linear.z  = right_button*controlEffort[THRUST] + (1.0 - right_button)*u_curr.linear.z;
        u_out.angular.x = right_button*controlEffort[ROLL]   + (1.0 - right_button)*u_curr.angular.x;
        u_out.angular.y = right_button*controlEffort[PITCH]  + (1.0 - right_button)*u_curr.angular.y;
        u_out.angular.z = right_button*controlEffort[YAW]    + (1.0 - right_button)*u_curr.angular.z;  
               
        // Check for integrator windup
        if(right_button)
        {
            checkForWindup(THRUST, u_out.linear.z,   desired_position.z,  curr_pos.z);
            checkForWindup(ROLL,   u_out.angular.x,  desired_position.y,  curr_pos.y);
            checkForWindup(PITCH,  u_out.angular.y,  desired_position.x,  curr_pos.x);
            checkForWindup(YAW,    u_out.angular.z,  0.0,                 curr_yaw);
        }
    
        // FOR TUNING/DEBUGGING. uncomment to turn off controller on a given axes
        //u_out.linear.z  = u_curr.linear.z;    // Thrust
        //u_out.linear.z = 0.15;
        //u_out.angular.x = u_curr.angular.x;   // Roll
        //u_out.angular.y = u_curr.angular.y;   // Pitch
        //u_out.angular.z = u_curr.angular.z;   // Yaw
        
        u_pub.publish(u_out);
        loop_rate.sleep();
    }   
}

// function to calculate control effort on a given axis
void calculateControlEffort(int index, double desPoint, double currPoint, double currVel)
{
    // Thrust is the force along the robot's z-axis
    propErr[index]  = Kp[index]*(desPoint - currPoint);
    intErr[index]  += Ki[index]*(desPoint - currPoint);
    derivErr[index] = Kd[index]*(-1.0*currVel);
    
    controlEffort[index] = propErr[index] + intErr[index] + derivErr[index];
}
    
// function to check for integral windup on a given axis
void checkForWindup(int index, double& control, double desPoint, double currPoint)
{
    if ( control > 1.0 )
    {
        control = 1.0;
        intErr[index] -= Ki[index]*(desPoint - currPoint);
    }
    else if ( control < -1.0 )
    {
        control = -1.0;
        intErr[index] -= Ki[index]*(desPoint - currPoint);
    }
}
