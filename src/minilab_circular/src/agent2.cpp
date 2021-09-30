#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Float64.h>

//
geometry_msgs::Pose2D current_pose_agent1;
ros::Time tk1_2;
float px_agent1,py_agent1,vx_agent1;
void odomCallbackagent1(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent1.x = msg->pose.pose.position.x;
    current_pose_agent1.y = msg->pose.pose.position.y;
    tk1_2 = msg->header.stamp;
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
    
}

geometry_msgs::Pose2D current_pose_agent2;

ros::Time tk1_1;
float px_agent2,py_agent2,vx_agent2, angle2;
void odomCallbackagent2(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent2.x = msg->pose.pose.position.x;
    current_pose_agent2.y = msg->pose.pose.position.y;
    tk1_1 = msg->header.stamp;
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
    angle2=current_pose_agent2.theta;
    
}

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    double ux,ux_integral = 0.0;
    double uy,uy_integral = 0.0;
    ros::Time int_time,current_time,prev_time;
    ros::Duration delta_t;
    double dvxhat1_2,vxhat1_2=0,dpxhat1_2,pxhat1_2=0,z0=0;
    double dvxhat2_2,vxhat2_2=0,dpxhat2_2,pxhat2_2=0,z1=0;
   

    double dvyhat1_2,vyhat1_2=0,dpyhat1_2,pyhat1_2=0,z0y=0;
    double dvyhat2_2,vyhat2_2=0,dpyhat2_2,pyhat2_2=0,z1y=0;
 

    double theta=4,lembda=0.12;

    double angle=0, dangle_hat=0, angle_hat=0,e=0,dw_hat=0,w_hat=0,z3;
    double v=0,w=0;
    double frx1=0,fry1=0,fvx1=0,fvy1=0,dfx1=0,dfy1=0;
    double frx2=0,fry2=0,fvx2=0,fvy2=0,dfx2=0,dfy2=0;
   
    ROS_INFO("start");

    ros::init(argc, argv, "agent2");
    ros::NodeHandle n;
    
    ros::Subscriber sub_odometryagent2 = n.subscribe("/agent2/odom", 10, odomCallbackagent2);
    
    ros::Subscriber sub_odometry = n.subscribe("/agent1/odom", 10, odomCallbackagent1);
    ros::Publisher movement_pubagent2 = n.advertise<geometry_msgs::Twist>("/agent2/cmd_vel",10); 

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
	frx1=2*cos(0.1*t.toSec());//4;//
	fry1=2*sin(0.1*t.toSec());//4;//
	fvx1=-.2*sin(0.1*t.toSec());//0;//
	fvy1=.2*cos(0.1*t.toSec());//0;//
	dfx1=-.02*cos(0.1*t.toSec());//0;
	dfy1=-.02*sin(0.1*t.toSec());0;//

	frx2=2*cos(0.1*t.toSec()+(2*PI)/3);//4;//
	fry2=2*sin(0.1*t.toSec()+(2*PI)/3);//-4;//
	fvx2=-.2*sin(0.1*t.toSec()+(2*PI)/3);//0;//
	fvy2=.2*cos(0.1*t.toSec()+(2*PI)/3);//0;//
	dfx2=-.02*cos(0.1*t.toSec()+(2*PI)/3);//0;//
	dfy2=-.02*sin(0.1*t.toSec()+(2*PI)/3);//0;//
////////////////////////////////////////////////////////////////////////////////
////////////// x-axis////////////////////////////////////////////////////////
        //Observer to estimate agent1's states
        z0=exp(-2*theta*(t.toSec()-tk1_2.toSec()))*(pxhat1_2-px_agent1);
        dvxhat1_2=dfx1-theta*theta*z0;
	dpxhat1_2=vxhat1_2-2*theta*z0;
	vxhat1_2 +=dvxhat1_2*delta_t.toSec(); // estimated velocity of the agent1
	pxhat1_2 +=dpxhat1_2*delta_t.toSec();// estimated position of the agent1
        //Observer to estimate agent2's states
        z1=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(pxhat2_2-px_agent2);
        dvxhat2_2=dfx2-theta*theta*z1;
	dpxhat2_2=vxhat2_2-2*theta*z1;
	vxhat2_2 +=dvxhat2_2*delta_t.toSec(); // estimated velocity of the agent2
	pxhat2_2 +=dpxhat2_2*delta_t.toSec();// estimated position of the agent2

	
////////////////////////////////////////////////////////////////////////////////
//////////////              y-axis                   ///////////////////////////
        //Observer to estimate agent1's states
        z0y=exp(-2*theta*(t.toSec()-tk1_2.toSec()))*(pyhat1_2-py_agent1);
        dvyhat1_2=dfy1-theta*theta*z0y;
	dpyhat1_2=vyhat1_2-2*theta*z0y;
	vyhat1_2 +=dvyhat1_2*delta_t.toSec(); // estimated velocity of the agent1
	pyhat1_2 +=dpyhat1_2*delta_t.toSec();// estimated position of the agent1
        //Observer to estimate agent2's states
        z1y=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(pyhat2_2-py_agent2);
        dvyhat2_2=dfy2-theta*theta*z1y;
	dpyhat2_2=vyhat2_2-2*theta*z1y;
	vyhat2_2 +=dvyhat2_2*delta_t.toSec(); // estimated velocity of the agent2
	pyhat2_2 +=dpyhat2_2*delta_t.toSec();// estimated position of the agent2
////////////////////////////////////////////////////////////////////////////////////////
////////////////// angle///////////////////////////
	z3=exp(-2*theta*(t.toSec()-tk1_1.toSec()))*(angle_hat-angle2);
        dw_hat=-theta*theta*z3;
	dangle_hat=dw_hat-2*theta*z3;
	w_hat +=dw_hat*delta_t.toSec(); 
	angle_hat +=dangle_hat*delta_t.toSec();// estimated angle of the agent2
////////////////////////////////////////////////////////////////
	

// calculating ux
	
	ux=dfx2-(lembda*lembda)*(pxhat2_2-pxhat1_2-frx2+frx1)-2*lembda*(vxhat2_2-vxhat1_2-fvx2+fvx1);
	ux_integral += ux*delta_t.toSec();//velocity
// calculating uy
	
	uy=dfy2-(lembda*lembda)*(pyhat2_2-pyhat1_2-fry2+fry1)-2*lembda*(vyhat2_2-vyhat1_2-fvy2+fvy1);
	uy_integral += uy*delta_t.toSec();//velocity
// calculating angles
/*	if (fabs(vyhat2_2)<0.0001 )
	{
		vyhat2_2=0;
	}
		if (fabs(vxhat2_2)<0.0001 )
	{
		vxhat2_2=0;
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
	angle_hat=atan2(vyhat2_2,vxhat2_2);
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
	//ROS_INFO("x anf y pos =[%f] [%f]",px_agent2,py_agent2);
	//ROS_INFO("x anf y ip =[%f] [%f]",ux_integral,uy_integral);
	//ROS_INFO(" =[%f]",px_agent2);
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
        movement_pubagent2.publish(move);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
