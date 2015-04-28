// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

//geometry_msgs::Vector3 u_out, curr_u;
geometry_msgs::Twist u_out, u_curr, u_des;

geometry_msgs::Vector3 curr_pos, prev_pos, hold_pos, curr_vel;
geometry_msgs::Vector3 desired_position, offset_position;

double right_button, reset_button, offset_button;

double curr_yaw, hold_yaw, curr_yaw_vel;

double Kxp, Kxi, Kxd;
double Kyp, Kyi, Kyd;
double Kzp, Kzi, Kzd;
double Kyawp, Kyawd;

double xpErr, xiErr, xdErr;
double ypErr, yiErr, ydErr;
double zpErr, ziErr, zdErr;
double yawpErr, yawdErr;

double a, b;

// Read xbox right trigger position
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	right_button = joy_msg_in.buttons[5];
}

// Read mocap position
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

// Read velocity
void vel_callback(const geometry_msgs::Vector3& vel_msg_in)
{
	curr_vel.x = vel_msg_in.x;
	curr_vel.y = vel_msg_in.y;
	curr_vel.z = vel_msg_in.z;
}

// Read current input
void u_callback(const geometry_msgs::Twist& u_msg_in)
{
    u_curr.angular.x = u_msg_in.angular.x;
    u_curr.angular.y = u_msg_in.angular.y;
    u_curr.angular.z = u_msg_in.angular.z;
    u_curr.linear.z  = u_msg_in.linear.z;
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
    
    ros::Publisher u_pub;
    u_pub = node.advertise<geometry_msgs::Twist>("new_u",1);

    ros::Subscriber joy_sub;
    joy_sub = node.subscribe("joy",1,joy_callback);
    
    ros::Subscriber u_sub;
    u_sub = node.subscribe("desired_u",1,u_callback);
    
    ros::Subscriber pos_sub, vel_sub, yaw_sub, yaw_vel_sub;
    pos_sub = node.subscribe("current_position",1,pos_callback);
    vel_sub = node.subscribe("current_velocity",1,vel_callback);
    yaw_sub = node.subscribe("current_yaw",1,yaw_callback);
    yaw_vel_sub = node.subscribe("current_rdot",1,yawVel_callback);
    
    ros::Subscriber desired_pos_sub;
    desired_pos_sub = node.subscribe("desired_position",1,desired_position_callback);
    
    // pulling in params from the launch file
    if(node.getParam("/pd_ratio",b)){;}	
    else
    {
	    ROS_ERROR("Set proportional to derivative ratio");
    	return 0;
    }
    
    if(node.getParam("/pi_ratio",a)){;}
    else
    {
        ROS_ERROR("Set proportional to integral ratio");
        return 0;
    }
    
    if( node.getParam("/x_gain",Kxp)){;}
    else
    {
        ROS_ERROR("Set X proportional gain");
        return 0;
    }
    //sets the gains from the ratio. Saves from setting all the gains
    Kxi = a*Kxp;
    Kxd = b*Kxp;


    if( node.getParam("/y_gain",Kyp)){;}
    else
    {
        ROS_ERROR("Set Y proportional gain");
        return 0;
    }
    Kyi = a*Kyp;
    Kyd = b*Kyp;

    if( node.getParam("/z_gain",Kzp)){;}
    else
    {
        ROS_ERROR("Set Z (linear) proportional gain");
        return 0;
    }
    
    Kzi = a*Kzp;
    Kzd = b*Kzp;
    
    //ROS_INFO("Kzi: %f \n Kzd: %f", Kzi, Kzd);
    
    if( node.getParam("/yaw_gain",Kyawp)){;}
    else
    {
        ROS_ERROR("Set yaw-rate proportional gain");
        return 0;
    }
    
    Kyawd = b*Kyawp;
    
    while(ros::ok())
    {
        ros::spinOnce();
                
        //seding out to make sure it reset
        ROS_INFO("new_position [%.4f, %.4f, %.4f]",curr_pos.x, curr_pos.y, curr_pos.z);
        //the left bumper button resets offset position
        //xbox button sets the current offset position
        
        if(!right_button) // if button not pressed, reset integral terms
        {
            xiErr = yiErr = ziErr = 0.0;
        }
        
        //Roll about the x axis
        //rotation x axis movement along y 
        xpErr  = Kxp*(desired_position.y - curr_pos.y);
        xiErr += Kxi*(desired_position.y - curr_pos.y);
        xdErr  = -Kxd*curr_vel.y;
        
        //Pitch about the y axis
        //movement along x        
        ypErr  = Kyp*(desired_position.x - curr_pos.x);
        yiErr += Kyi*(desired_position.x - curr_pos.x);
        ydErr  = -Kyd*curr_vel.x;

        //Error in the height (z)
        //controls thrust        
        zpErr  = Kzp*(desired_position.z - curr_pos.z);
        ziErr += Kzi*(desired_position.z - curr_pos.z);
        zdErr  = -Kzd*curr_vel.z;
        
        //actual yaw angle about zero 
        yawpErr = -Kyawp*curr_yaw;
        yawdErr = -Kyawd*curr_yaw_vel;
        
        // Roll is error about y position
        // if button pressed, getting applying the controller
        // else just input form joystick (new_u of joystick)
        u_out.angular.x = right_button*(xpErr + xiErr + xdErr) + (1.0 - right_button)*u_curr.angular.x;
        
        // Check for roll integrator windup
        if(right_button)
        {
        
            // check if it is saturated and cap it at -1 or 1
            if(u_out.angular.x > 1.0)
            {
                u_out.angular.x = 1.0;
                xiErr -= Kxi*(desired_position.y - curr_pos.y);
            }
            else if(u_out.angular.x < -1.0)
            {
                u_out.angular.x = -1.0;
                xiErr -= Kxi*(desired_position.y - curr_pos.y);
            }
            /*
            // this is here to try and damp out the overshoot
            if( (prev_pos.y < desired_position.y && curr_pos.y > desired_position.y) ||
                (prev_pos.y > desired_position.y && curr_pos.y < desired_position.y) )
            {
                xiErr = 0.0;
            }*/
        }
    
        // Pitch is error about x position
        u_out.angular.y = right_button*(ypErr + yiErr + ydErr) + (1.0 - right_button)*u_curr.angular.y;
        
        // Check for pitch integrator windup
        if(right_button)
        {
            if(u_out.angular.y > 1.0)
            {
                u_out.angular.y = 1.0;
                yiErr -= Kyi*(desired_position.x - curr_pos.x);
            }
            else if(u_out.angular.y < -1.0)
            {
                u_out.angular.y = -1.0;
                yiErr -= Kyi*(desired_position.x - curr_pos.x);
            }
            
             /*
            // this is here to try and damp out the overshoot
           
            if( (prev_pos.x < desired_position.x && curr_pos.x > desired_position.x) ||
                (prev_pos.x > desired_position.x && curr_pos.x < desired_position.x) )
            {
                yiErr = 0.0;
            }*/
        }
        
        // Thrust is error about z position
        u_out.linear.z = right_button*(zpErr + ziErr + zdErr) + (1.0 - right_button)*u_curr.linear.z;
        
        //ROS_INFO("u_out.z: %f", u_out.linear.z);
        
        // Check for thrust integrator windup
        if(right_button)
        {
            if(u_out.linear.z > 1.0)
            {
                u_out.linear.z = 1.0;
                ziErr -= Kzi*(desired_position.z - curr_pos.z);
            }
            else if(u_out.linear.z < -1.0)
            {
                u_out.linear.z = -1.0;
                ziErr -= Kzi*(desired_position.z - curr_pos.z);
            }
            /*
            // this is here to try and damp out the overshoot
            if( (prev_pos.z < desired_position.z && curr_pos.z > desired_position.z) ||
                (prev_pos.z > desired_position.z && curr_pos.z < desired_position.z) )
            {
                ziErr = 0.0;
            }*/
        }
        
        //ROS_INFO("%f, %f, %f", zpErr, ziErr, zdErr);
        
        // Yaw if rotation about z axis
        u_out.angular.z = right_button*(yawpErr + yawdErr) + (1.0 - right_button)*u_curr.angular.z;
        
        if(u_out.angular.z > 1.0)
        {
            u_out.angular.z = 1.0;
        }
        else if(u_out.angular.z < -1.0)
        {
            u_out.angular.z = -1.0;
        }
        
        // FOR TUNING/DEBUGGING. turn off controller on axes
        //u_out.angular.x = u_curr.angular.x;
        //u_out.angular.y = u_curr.angular.y;
        //u_out.angular.z = u_curr.angular.z;
        /*u_out.linear.z  = u_curr.linear.z;*/
        
        u_pub.publish(u_out);
        loop_rate.sleep();
    }
}
