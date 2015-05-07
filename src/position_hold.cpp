// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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
geometry_msgs::Vector3 current_pos, current_vel, previous_pos;
geometry_msgs::Vector3 desired_pos, offset_pos;
Eigen::Vector3f rot_desired_pos, rot_current_pos, rot_current_vel;

double curr_yaw, curr_yaw_vel;

// right bumper for turning on PID on position
// left bumper for setting the "zero" position of robot
// xbox button for cancelling relative positioning of robot
double right_button, reset_button, offset_button;
int joy_a, joy_b, startControl;

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
	joy_a = joy_msg_in.buttons[0];
	joy_b = joy_msg_in.buttons[1];
}

// Read mocap position, allow for relative positioning with left bumper and reset button
void pos_callback(const geometry_msgs::Vector3& pos_msg_in)
{
    if(reset_button)
    {
        offset_pos.x = 0.0;
        offset_pos.y = 0.0;
        offset_pos.z = 0.0;
    }
    else if(offset_button){
        offset_pos.x = pos_msg_in.x;
        offset_pos.y = pos_msg_in.y;
        offset_pos.z = pos_msg_in.z;
    }
	current_pos.x = pos_msg_in.x - offset_pos.x;
	current_pos.y = pos_msg_in.y - offset_pos.y;
	current_pos.z = pos_msg_in.z - offset_pos.z;
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
	current_vel.x = vel_msg_in.x;
	current_vel.y = vel_msg_in.y;
	current_vel.z = vel_msg_in.z;
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
    desired_pos.x = des_pos_msg_in.x;
    desired_pos.y = des_pos_msg_in.y;
    desired_pos.z = des_pos_msg_in.z;
}

// function to calculate control effort on a given axis
void calculateControlEffort(int index, double desPoint, double currPoint, double currVel);

// function to check for integral windup on a given axis
void checkForWindup(int index, double& control, double desPoint, double currPoint, double prevPoint);

// function to calculate rotated desired and current positions
void rotatePositions();

int main(int argc, char** argv)
{
    // ROS Initialization stuff
    ros::init(argc,argv,"position_hold");
    ros::NodeHandle node;
    ros::Rate loop_rate(100);
   
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
    node.getParam("thrustP",Kp[THRUST]);
    node.getParam("thrustI",Ki[THRUST]);
    node.getParam("thrustD",Kd[THRUST]);
    node.getParam("rollP",  Kp[ROLL]);
    node.getParam("rollI",  Ki[ROLL]);
    node.getParam("rollD",  Kd[ROLL]);
    node.getParam("pitchP", Kp[PITCH]);
    node.getParam("pitchI", Ki[PITCH]);
    node.getParam("pitchD", Kd[PITCH]);
    node.getParam("yawP",   Kp[YAW]);
    node.getParam("yawI",   Ki[YAW]);
    node.getParam("yawD",   Kd[YAW]);

    bool flag = false;
    startControl = 0;
    // Loop until node closed or some ROS crash/error
    while(ros::ok())
    {        
        // Set in old position for integral windup check
        previous_pos.x = current_pos.x;
        previous_pos.y = current_pos.y;
        previous_pos.z = current_pos.z;
        
        // Read in subscribed messages
        ros::spinOnce();
            
        if( (joy_a && !startControl) || (joy_b && startControl) )
        {
            startControl = !startControl;
        } 
        if(startControl)
        {
            right_button = 1;
        }
        else
        {
            right_button = 0;
        }   
        //seding out to make sure it reset
        // Send every other loop to slow down scree print
        /*flag = !flag;
        if(flag)
        {
            ROS_INFO("new_position [%.4f, %.4f, %.4f]",current_pos.x, current_pos.y, current_pos.z);
        }*/

        if(!right_button) // if button not pressed, reset integral terms to prevent windup
        {
            for(int i = 0; i < 4; i++)
            {
                intErr[i] = 0.0;
            }
        }
        
        // DOES NOT WORK!!!!!
        // Rotate global axes into robot's frame (yaw only)
        rotatePositions();
        // ^^ DOES NOT WORK!!!!!
        
        // Calculate PID control effort for each axis
        calculateControlEffort(THRUST,  rot_desired_pos[2],  rot_current_pos[2],  rot_current_vel[2]);
        calculateControlEffort(ROLL,    rot_desired_pos[1],  rot_current_pos[1],  rot_current_vel[1]);
        calculateControlEffort(PITCH,   rot_desired_pos[0],  rot_current_pos[0],  rot_current_vel[0]);
        calculateControlEffort(YAW,     0.0,                 curr_yaw,            curr_yaw_vel);
        
        // Apply thrust, roll, pitch, and yaw control
        u_out.linear.z  = right_button*controlEffort[THRUST] + (1.0 - right_button)*u_curr.linear.z;
        u_out.angular.x = right_button*controlEffort[ROLL]   + (1.0 - right_button)*u_curr.angular.x;
        u_out.angular.y = right_button*controlEffort[PITCH]  + (1.0 - right_button)*u_curr.angular.y;
        u_out.angular.z = right_button*controlEffort[YAW]    + (1.0 - right_button)*u_curr.angular.z;  
               
        // Check for integrator windup
        if(right_button)
        {
            checkForWindup(THRUST, u_out.linear.z,   rot_desired_pos[2],  rot_current_pos[2], previous_pos.z);
            checkForWindup(ROLL,   u_out.angular.x,  rot_desired_pos[1],  rot_current_pos[1], previous_pos.y);
            checkForWindup(PITCH,  u_out.angular.y,  rot_desired_pos[0],  rot_current_pos[0], previous_pos.x);
            checkForWindup(YAW,    u_out.angular.z,  0.0,                 curr_yaw,           0.0);
        }
    
        // FOR TUNING/DEBUGGING. uncomment to turn off controller on a given axes
        //u_out.linear.z  = u_curr.linear.z;    // Thrust
        //u_out.linear.z = 0.15;
        //u_out.angular.x = u_curr.angular.x;   // Roll
        //u_out.angular.y = u_curr.angular.y;   // Pitch
        //u_out.angular.z = u_curr.angular.z;   // Yaw
        //u_out.angular.z = 0.5;
        
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
void checkForWindup(int index, double& control, double desPoint, double currPoint, double prevPoint)
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
    
    if(index == ROLL || index == PITCH)
    {
        if( (prevPoint < desPoint && currPoint > desPoint) || (prevPoint > desPoint && currPoint < desPoint) )
        {
            intErr[index] = 0.0; 
        }
    }
}

// function to calculate rotated desired and current positions
// DOES NOT WORK
void rotatePositions()
{
    Eigen::Rotation2D<float> rot2(curr_yaw);
    rot2 = rot2.inverse();
    //Eigen::Rotation2D<float> rot2(0.0);
    
    Eigen::Vector2f des_pos, curr_pos, curr_vel;
    des_pos << desired_pos.x, desired_pos.y;
    curr_pos << current_pos.x, current_pos.y;
    curr_vel << current_vel.x, current_vel.y;
    
    rot_desired_pos << (rot2*des_pos)[0],  (rot2*des_pos)[1],  desired_pos.z;
    rot_current_pos << (rot2*curr_pos)[0], (rot2*curr_pos)[1], current_pos.z;
    rot_current_vel << (rot2*curr_vel)[0], (rot2*curr_vel)[1], current_vel.z;
}
