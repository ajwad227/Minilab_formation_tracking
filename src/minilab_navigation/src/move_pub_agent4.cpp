#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose2D current_pose_agent1;
ros::Time tk1_4;
float px_agent1,py_agent1,vx_agent1;
void odomCallbackagent1(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent1.x = msg->pose.pose.position.x;
    current_pose_agent1.y = msg->pose.pose.position.y;
    tk1_4 = msg->header.stamp;
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
//////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose2D current_pose_agent3;
ros::Time tk3_4;
float px_agent3,py_agent3,vx_agent3;
void odomCallbackagent3(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent3.x = msg->pose.pose.position.x;
    current_pose_agent3.y = msg->pose.pose.position.y;
    tk3_4 = msg->header.stamp;
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
}
///////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose2D current_pose_agent4;
ros::Time tk4_4;

float px_agent4,py_agent4,vx_agent4;
void odomCallbackagent4(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent4.x = msg->pose.pose.position.x;
    current_pose_agent4.y = msg->pose.pose.position.y;
    tk4_4 = msg->header.stamp;
    px_agent4=current_pose_agent4.x;
    py_agent4=current_pose_agent4.y;
    vx_agent4 = msg->twist.twist.linear.x;
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
    current_pose_agent4.theta = yaw;
    
}
///////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    double u,p,v,u_integral = 0;
    ros::Time int_time,current_time,prev_time;
    ros::Duration delta_t;
    double dvhat1_4,vhat1_4=0,dpxhat1_4,pxhat1_4=0,z1=0;
    double dvhat3_4,vhat3_4=0,dpxhat3_4,pxhat3_4=0,z3=0;     
    double dvhat4_4,vhat4_4=0,dpxhat4_4,pxhat4_4=0,z4=0;
       
    double theta=3, lembda=.4; // Gains
    ROS_INFO("start");
    ros::init(argc, argv, "move_pub_agent4");
    ros::NodeHandle n;
    
    ros::Subscriber sub_odometryagent4 = n.subscribe("/agent4/odom", 10, odomCallbackagent4);
    ros::Subscriber sub_odometryagent3 = n.subscribe("/agent3/odom", 10, odomCallbackagent3);
    ros::Subscriber sub_odometryagent1 = n.subscribe("/agent1/odom", 10, odomCallbackagent1);
    ros::Publisher movement_pubagent4 = n.advertise<geometry_msgs::Twist>("/agent4/cmd_vel",10); 
    //for sensors the value after , should be higher to get a more accurate result (queued)
    
    ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement
   
    
    //move
    ros::Time start1 = ros::Time::now();
    while(ros::ok())
    {
	geometry_msgs::Twist move;
	ros::Time t = ros::Time::now();
   	// calculate delta_t
	delta_t = t - prev_time;
	prev_time = ros::Time::now();

        //Observer to estimate agent1's states
        z1=exp(-2*theta*(t.toSec()-tk1_4.toSec()))*(pxhat1_4-px_agent1);
        dvhat1_4=-theta*theta*z1;
	vhat1_4 +=dvhat1_4*delta_t.toSec();
	dpxhat1_4=vhat1_4-2*theta*z1;
	pxhat1_4 +=dpxhat1_4*delta_t.toSec();

        //Observer to estimate agent3's states
        z3=exp(-2*theta*(t.toSec()-tk3_4.toSec()))*(pxhat3_4-px_agent3);
        dvhat3_4=-theta*theta*z3;
	vhat3_4 +=dvhat3_4*delta_t.toSec();
	dpxhat3_4=vhat3_4-2*theta*z3;
	pxhat3_4 +=dpxhat3_4*delta_t.toSec();
        
	//Observer to estimate agent4's states
        z4=exp(-2*theta*(t.toSec()-tk4_4.toSec()))*(pxhat4_4-px_agent4);
        dvhat4_4=-theta*theta*z4;
	vhat4_4 +=dvhat4_4*delta_t.toSec();
	dpxhat4_4=vhat4_4-2*theta*z4;
	pxhat4_4 +=dpxhat4_4*delta_t.toSec();

	// calculating u
	//p=(pxhat4_4-pxhat1_4);
	//v=(vhat4_4-vhat1_4);
	u=-(lembda*lembda)*(pxhat4_4-pxhat1_4+pxhat4_4-pxhat3_4)-lembda*(vhat4_4-vhat3_4+vhat4_4-vhat3_4);
	u_integral += u*delta_t.toSec();
        
	//velocity controls
        move.linear.x = u_integral; //speed value m/s
        move.angular.z = 0;
        movement_pubagent4.publish(move);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
