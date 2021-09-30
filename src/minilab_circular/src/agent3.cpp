#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Float64.h>

//
geometry_msgs::Pose2D current_pose_agent2;
ros::Time tk0_1;
float px_agent2,py_agent2,vx_agent2;
void odomCallbackagent2(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent2.x = msg->pose.pose.position.x;
    current_pose_agent2.y = msg->pose.pose.position.y;
    tk0_1 = msg->header.stamp;
    px_agent2=current_pose_agent2.x;
    py_agent2=current_pose_agent2.y;
    vx_agent2 = msg->twist.twist.linear.x;
   
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
    current_pose_agent2.theta = yaw;
    
}

geometry_msgs::Pose2D current_pose_agent3;

ros::Time tk1_1;
float px_agent3,py_agent3,vx_agent3, angle2;
void odomCallbackagent3(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent3.x = msg->pose.pose.position.x;
    current_pose_agent3.y = msg->pose.pose.position.y;
    tk1_1 = msg->header.stamp;
    px_agent3=current_pose_agent3.x;
    py_agent3=current_pose_agent3.y;
    vx_agent3 = msg->twist.twist.linear.x;
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
    current_pose_agent3.theta = yaw;
    angle2=current_pose_agent3.theta;
    
}

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    double ux,ux_integral = 0.0;
    double uy,uy_integral = 0.0;
    ros::Time int_time,current_time,prev_time;
    ros::Duration delta_t;
    double dvxhat2_3,vxhat2_3=0,dpxhat2_3,pxhat2_3=0,z0=0;
    double dvxhat3_3,vxhat3_3=0,dpxhat3_3,pxhat3_3=0,z1=0;
   

    double dvyhat0_1,vyhat0_1=0,dpyhat1_2,pyhat1_2=0,z0y=0;
    double dvyhat2_2,vyhat2_2=0,dpyhat2_2,pyhat2_2=0,z1y=0;
 

    double theta=4,lembda=0.12;

    double angle=0, dangle_hat=0, angle_hat=0,e=0,dw_hat=0,w_hat=0,z3;
    double v=0,w=0;
    double frx3=0,fry3=0,fvx3=0,fvy3=0,dfx3=0,dfy3=0;
    double frx2=0,fry2=0,fvx2=0,fvy2=0,dfx2=0,dfy2=0;
   
    ROS_INFO("start");

    ros::init(argc, argv, "agent3");
    ros::NodeHandle n;
    
    ros::Subscriber sub_odometryagent3 = n.subscribe("/agent3/odom", 10, odomCallbackagent3);
    
    ros::Subscriber sub_odometry = n.subscribe("/agent2/odom", 10, odomCallbackagent2);
    ros::Publisher movement_pubagent3 = n.advertise<geometry_msgs::Twist>("/agent3/cmd_vel",10); 

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
// formation pattern
	frx2=2*cos(0.1*t.toSec()+(2*PI)/3);//4;//
	fry2=2*sin(0.1*t.toSec()+(2*PI)/3);//-4;//
	fvx2=-.2*sin(0.1*t.toSec()+(2*PI)/3);//0;//
	fvy2=.2*cos(0.1*t.toSec()+(2*PI)/3);//0;//
	dfx2=-.02*cos(0.1*t.toSec()+(2*PI)/3);//
	dfy2=-.02*sin(0.1*t.toSec()+(2*PI)/3);//0;

	frx3=2*cos(0.1*t.toSec()+(4*PI)/3);//4;//
	fry3=2*sin(0.1*t.toSec()+(4*PI)/3);//-4;//
	fvx3=-.2*sin(0.1*t.toSec()+(4*PI)/3);//0;//
	fvy3=.2*cos(0.1*t.toSec()+(4*PI)/3);//0;//
	dfx3=-.02*cos(0.1*t.toSec()+(4*PI)/3);//0;//
	dfy3=-.02*sin(0.1*t.toSec()+(4*PI)/3);//0;//
////////////////////////////////////////////////////////////////////////////////
////////////// x-axis////////////////////////////////////////////////////////
        //Observer to estimate agent2's states
        z0=exp(-2*theta*(t.toSec()-tk0_1.toSec()))*(pxhat2_3-px_agent2);
        dvxhat2_3=dfx2-theta*theta*z0;
	dpxhat2_3=vxhat2_3-2*theta*z0;
	vxhat2_3 +=dvxhat2_3*delta_t.toSec(); // estimated velocity of the agent2
	pxhat2_3 +=dpxhat2_3*delta_t.toSec();// estimated position of the agent2
        //Observer to estimate agent3's states
        z1=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(pxhat3_3-px_agent3);
        dvxhat3_3=dfx3-theta*theta*z1;
	dpxhat3_3=vxhat3_3-2*theta*z1;
	vxhat3_3 +=dvxhat3_3*delta_t.toSec(); // estimated velocity of the agent3
	pxhat3_3 +=dpxhat3_3*delta_t.toSec();// estimated position of the agent3

	
////////////////////////////////////////////////////////////////////////////////
//////////////              y-axis                   ///////////////////////////
        //Observer to estimate agent2's states
        z0y=exp(-2*theta*(t.toSec()-tk0_1.toSec()))*(pyhat1_2-py_agent2);
        dvyhat0_1=dfy2-theta*theta*z0y;
	dpyhat1_2=vyhat0_1-2*theta*z0y;
	vyhat0_1 +=dvyhat0_1*delta_t.toSec(); // estimated velocity of the agent2
	pyhat1_2 +=dpyhat1_2*delta_t.toSec();// estimated position of the agent2
        //Observer to estimate agent3's states
        z1y=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(pyhat2_2-py_agent3);
        dvyhat2_2=dfy3-theta*theta*z1y;
	dpyhat2_2=vyhat2_2-2*theta*z1y;
	vyhat2_2 +=dvyhat2_2*delta_t.toSec(); // estimated velocity of the agent3
	pyhat2_2 +=dpyhat2_2*delta_t.toSec();// estimated position of the agent3
////////////////////////////////////////////////////////////////////////////////////////
////////////////// angle///////////////////////////
	z3=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(angle_hat-angle2);
        dw_hat=-theta*theta*z3;
	dangle_hat=dw_hat-2*theta*z3;
	w_hat +=dw_hat*delta_t.toSec(); 
	angle_hat +=dangle_hat*delta_t.toSec();// estimated angle of the agent3
////////////////////////////////////////////////////////////////
	

// calculating ux
	
	ux=dfx3-(lembda*lembda)*(pxhat3_3-pxhat2_3-frx3+frx2)-2*lembda*(vxhat3_3-vxhat2_3-fvx3+fvx2);
	ux_integral += ux*delta_t.toSec();//velocity
// calculating uy
	
	uy=dfy3-(lembda*lembda)*(pyhat2_2-pyhat1_2-fry3+fry2)-2*lembda*(vyhat2_2-vyhat0_1-fvy3+fvy2);
	uy_integral += uy*delta_t.toSec();//velocity
// calculating angles
/*	if (fabs(vyhat2_2)<0.0001 )
	{
		vyhat2_2=0;
	}
		if (fabs(vxhat3_3)<0.0001 )
	{
		vxhat3_3=0;
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
	angle_hat=atan2(vyhat2_2,vxhat3_3);
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
        ROS_INFO(" actual  hat= [%f] [%f]",angle2,angle_hat);
	//ROS_INFO("x anf y pos =[%f] [%f]",px_agent3,py_agent3);
	//ROS_INFO("x anf y ip =[%f] [%f]",ux_integral,uy_integral);
	//ROS_INFO(" =[%f]",px_agent3);
//calculating linear and angular velocities
	//v=sqrt(pow(ux_integral,2)+pow(uy_integral,2))	;
	v=(cos(angle) * ux_integral+ sin(angle) * uy_integral);
	//w= 0.5*( -1/0.12 * sin(angle_hat) * ux_integral + 1/0.12 * cos(angle_hat) * uy_integral);
	w=2*e;
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
        movement_pubagent3.publish(move);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
