// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

geometry_msgs::Vector3 u_out, curr_u;
geometry_msgs::Vector3 curr_pos, hold_pos, curr_vel;
geometry_msgs::Vector3 desired_position;
std_msgs::Float32 yaw_out, curr_u_yaw;
double curr_yaw, hold_yaw, curr_yaw_vel;

double Kp = 0.2;
double Kpx = Kp, Kpy = Kp, Kpz = 2.0*Kp, Kpyaw = Kp;

double Kd = 0.1;
double Kdx = Kd, Kdy = Kd, Kdz = Kd, Kdyaw = 0.5*Kd;

// Read mocap position
void pos_callback(const geometry_msgs::Vector3& pos_msg_in)
{
	curr_pos.x = pos_msg_in.x;
	curr_pos.y = pos_msg_in.y;
	curr_pos.z = pos_msg_in.z;
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

// Read velocity
void vel_callback(const geometry_msgs::Vector3& vel_msg_in)
{
	curr_vel.x = vel_msg_in.x;
	curr_vel.y = vel_msg_in.y;
	curr_vel.z = vel_msg_in.z;
}

// Read current input
void u_callback(const geometry_msgs::Vector3& u_msg_in)
{
    curr_u.x = u_msg_in.x;
    curr_u.y = u_msg_in.y;
    curr_u.z = u_msg_in.z;
}

// Read yaw input
void u_yaw_callback(const std_msgs::Float32& u_yaw_msg_in)
{
    curr_u_yaw = u_yaw_msg_in;
}

// Read desired position
void desired_position_callback(const geometry_msgs::Vector3& des_pos_msg_in)
{
    desired_position.x = des_pos_msg_in.x;
    desired_position.y = des_pos_msg_in.y;
    desired_position.z = des_pos_msg_in.z;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"position_hold");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    ros::Publisher u_pub,yaw_pub;
    u_pub = node.advertise<geometry_msgs::Vector3>("new_u",1);
    yaw_pub = node.advertise<std_msgs::Float32>("new_yaw",1);

    ros::Subscriber u_sub, u_yaw_sub;
    u_sub = node.subscribe("desired_u",1,u_callback);
    u_yaw_sub = node.subscribe("desired_yaw",1,u_yaw_callback);
    
    ros::Subscriber pos_sub, vel_sub, yaw_sub, yaw_vel_sub;
    pos_sub = node.subscribe("current_position",1,pos_callback);
    vel_sub = node.subscribe("current_velocity",1,vel_callback);
    yaw_sub = node.subscribe("current_yaw",1,yaw_callback);
    yaw_vel_sub = node.subscribe("current_rdot",1,yawVel_callback);
    
    ros::Subscriber desired_pos_sub;
    desired_pos_sub = node.subscribe("desired_position",1,desired_position_callback);
    
    double eps = 0.1;
    while(ros::ok())
    {
        u_out.x = curr_u.x;
        u_out.y = curr_u.y;
        u_out.z = curr_u.z;
        yaw_out = curr_u_yaw;
        
        if(u_out.x < eps && u_out.x > -eps)
            u_out.x = Kpx*(desired_position.x - curr_pos.x) - Kdx*curr_vel.x;
        if(u_out.y < eps && u_out.y > -eps)
            u_out.y = Kpy*(desired_position.y - curr_pos.y) - Kdy*curr_vel.y;
        if(u_out.z < eps && u_out.z > -eps)
            u_out.z = Kpz*(desired_position.z - curr_pos.z) - Kdz*curr_vel.z;
        if(yaw_out.data < eps && yaw_out.data > -eps)
            yaw_out.data = -Kpyaw*curr_yaw - Kdyaw*curr_yaw_vel;
        u_pub.publish(u_out);
        yaw_pub.publish(yaw_out);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
