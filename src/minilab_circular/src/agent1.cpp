#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Float64.h>

//
geometry_msgs::Pose2D current_pose_leader;
ros::Time tk0_1;
float px_leader,py_leader,vx_leader;
void odomCallbackLeader(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_leader.x = msg->pose.pose.position.x;
    current_pose_leader.y = msg->pose.pose.position.y;
    tk0_1 = msg->header.stamp;
    px_leader=current_pose_leader.x;
    py_leader=current_pose_leader.y;
    vx_leader = msg->twist.twist.linear.x;
   
    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // angular position
    current_pose_leader.theta = yaw;
    
}

geometry_msgs::Pose2D current_pose_agent1;

ros::Time tk1_1;
float px_agent1,py_agent1,vx_agent1, angle1;
void odomCallbackagent1(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent1.x = msg->pose.pose.position.x;
    current_pose_agent1.y = msg->pose.pose.position.y;
    tk1_1 = msg->header.stamp;
    px_agent1=current_pose_agent1.x;
    py_agent1=current_pose_agent1.y;
    vx_agent1 = msg->twist.twist.linear.x;
    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // angular position
    current_pose_agent1.theta = yaw;
    angle1=current_pose_agent1.theta;
    
}

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    double ux,ux_integral = 0.0;
    double uy,uy_integral = 0.0;
    ros::Time int_time,current_time,prev_time;
    ros::Duration delta_t;
    double dvxhat0_1,vxhat0_1=0,dpxhat0_1,pxhat0_1=0,z0=0;
    double dvxhat1_1,vxhat1_1=0,dpxhat1_1,pxhat1_1=0,z1=0;
   

    double dvyhat0_1,vyhat0_1=0,dpyhat0_1,pyhat0_1=0,z0y=0;
    double dvyhat1_1,vyhat1_1=0,dpyhat1_1,pyhat1_1=0,z1y=0;
 

    double theta=4,lembda=0.2;

    double angle=0, dangle_hat=0, angle_hat=0,e=0,dw_hat=0,w_hat=0,z3;
    double v=0,w=0;
    double frx1=0,fry1=0,fvx1=0,fvy1=0,dfx=0,dfy=0;
   
    ROS_INFO("start");

    ros::init(argc, argv, "agent1");
    ros::NodeHandle n;
    
    ros::Subscriber sub_odometryAgent1 = n.subscribe("/agent1/odom", 10, odomCallbackagent1);
    
    ros::Subscriber sub_odometry = n.subscribe("/leader/odom", 10, odomCallbackLeader);
    ros::Publisher movement_pubAgent1 = n.advertise<geometry_msgs::Twist>("/agent1/cmd_vel",10); 

    //for sensors the value after , should be higher to get a more accurate result (queued)
    
    ros::Rate rate(100); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement
     

    //move Agent 1

    ROS_INFO("move");
    
    while(ros::ok())
    {
	geometry_msgs::Twist move;
	std_msgs::Float64 input;
	ros::Time t = ros::Time::now();
   	// calculate delta_t
	delta_t = t - prev_time;
	prev_time = ros::Time::now();
////////////////////////////////////////////////////////////////////////////////
////////////// x-axis////////////////////////////////////////////////////////
        //Observer to estimate leader's states
        z0=exp(-2*theta*(t.toSec()-tk0_1.toSec()))*(pxhat0_1-px_leader);
        dvxhat0_1=-theta*theta*z0;
	dpxhat0_1=vxhat0_1-2*theta*z0;
	vxhat0_1 +=dvxhat0_1*delta_t.toSec(); // estimated velocity of the leader
	pxhat0_1 +=dpxhat0_1*delta_t.toSec();// estimated position of the leader
        //Observer to estimate agent1's states
        z1=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(pxhat1_1-px_agent1);
        dvxhat1_1=-theta*theta*z1;
	dpxhat1_1=vxhat1_1-2*theta*z1;
	vxhat1_1 +=dvxhat1_1*delta_t.toSec(); // estimated velocity of the agent1
	pxhat1_1 +=dpxhat1_1*delta_t.toSec();// estimated position of the agent1

	
////////////////////////////////////////////////////////////////////////////////
//////////////              y-axis                   ///////////////////////////
        //Observer to estimate leader's states
        z0y=exp(-2*theta*(t.toSec()-tk0_1.toSec()))*(pyhat0_1-py_leader);
        dvyhat0_1=-theta*theta*z0y;
	dpyhat0_1=vyhat0_1-2*theta*z0y;
	vyhat0_1 +=dvyhat0_1*delta_t.toSec(); // estimated velocity of the leader
	pyhat0_1 +=dpyhat0_1*delta_t.toSec();// estimated position of the leader
        //Observer to estimate agent1's states
        z1y=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(pyhat1_1-py_agent1);
        dvyhat1_1=-theta*theta*z1y;
	dpyhat1_1=vyhat1_1-2*theta*z1y;
	vyhat1_1 +=dvyhat1_1*delta_t.toSec(); // estimated velocity of the agent1
	pyhat1_1 +=dpyhat1_1*delta_t.toSec();// estimated position of the agent1
////////////////////////////////////////////////////////////////////////////////////////
////////////////// angle///////////////////////////
	z3=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(angle_hat-angle1);
        dw_hat=-theta*theta*z3;
	dangle_hat=dw_hat-2*theta*z3;
	w_hat +=dw_hat*delta_t.toSec(); 
	angle_hat +=dangle_hat*delta_t.toSec();// estimated angle of the agent1
////////////////////////////////////////////////////////////////
	
// formation pattern
	frx1=2*cos(0.1*t.toSec());//4;//
	fry1=2*sin(0.1*t.toSec());//4;//
	fvx1=-0.2*sin(0.1*t.toSec());//0;//
	fvy1=0.2*cos(0.1*t.toSec());//0;//
	dfx=-0.02*cos(0.1*t.toSec());//0;//
	dfy=-0.02*sin(0.1*t.toSec());//0;//
// calculating ux
	
	ux=dfx-(lembda*lembda)*(pxhat1_1-pxhat0_1-frx1)-2*lembda*(vxhat1_1-vxhat0_1-fvx1);
	ux_integral += ux*delta_t.toSec();//velocity
// calculating uy
	
	uy=dfy-(lembda*lembda)*(pyhat1_1-pyhat0_1-fry1)-2*lembda*(vyhat1_1-vyhat0_1-fvy1);
	uy_integral += uy*delta_t.toSec();//velocity
// calculating angles
/*	if (fabs(vyhat1_1)<0.0001 )
	{
		vyhat1_1=0;
	}
		if (fabs(vxhat1_1)<0.0001 )
	{
		vxhat1_1=0;
	}
		if (fabs(ux_integral)<0.0001 )
	{
		ux_integral=0;
	}
		if (fabs(uy_integral)<0.0001 )
	{
		uy_integral=0;
	}
		if (fabs(angle_hat)<0.01 )
	{
		angle_hat=0;
	}*/

	angle_hat=atan2(vyhat1_1,vxhat1_1);
	angle=atan2(uy_integral,ux_integral);
	e=angle-angle_hat;
	
	if (e>PI)
	{
		e-=2*PI;
	}	
 	if (e<-PI)
	{
		e+=2*PI;
	}
	if (fabs(e)<0.01)
	{
		e=0;
	}
        ROS_INFO(" desired and hat=[%f] [%f]",angle,angle_hat);
	//ROS_INFO("x anf y pos =[%f] [%f]",px_agent1,py_agent1);
	//ROS_INFO("x anf y ip =[%f] [%f]",ux_integral,uy_integral);
	//ROS_INFO(" =[%f]",px_agent1);
//calculating linear and angular velocities
	//v=sqrt(pow(ux_integral,2)+pow(uy_integral,2))	;
	v= (cos(angle) * ux_integral+ sin(angle) * uy_integral);
	//w= 0.5*( -1/0.12 * sin(angle_hat) * ux_integral + 1/0.12 * cos(angle_hat) * uy_integral);
	w=1.7*e;
	//w=2*e+2*sin(e)*cos(e);
	/*if (v>1)
	{
		v=1;
	}	
 	if (v<-1)
	{
		v=-1;
	}
 	if (w>1)
	{
		w=1;
	}	
 	if (w<-1)
	{
		w=-1;
	}*/
ROS_INFO("v w e =[%f] [%f] [%f]",v,w,e);
//velocity controls
        move.linear.x = v; //speed value m/s
        move.angular.z = w;
        movement_pubAgent1.publish(move);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
