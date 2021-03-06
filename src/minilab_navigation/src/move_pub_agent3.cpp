#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


geometry_msgs::Pose2D current_pose_agent2;
ros::Time tk2_3;
float px_agent2,py_agent2,vx_agent2;
void odomCallbackagent2(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent2.x = msg->pose.pose.position.x;
    current_pose_agent2.y = msg->pose.pose.position.y;
    tk2_3 = msg->header.stamp;
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
ros::Time tk3_3;
float px_agent3,py_agent3,vx_agent3;
void odomCallbackagent3(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_agent3.x = msg->pose.pose.position.x;
    current_pose_agent3.y = msg->pose.pose.position.y;
    tk3_3 = msg->header.stamp;
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
    current_pose_agent2.theta = yaw;
}

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    double u,u_integral = 0;
    ros::Time int_time,current_time,prev_time;
    ros::Duration delta_t;
    double dvhat2_3,vhat2_3=0,dpxhat2_3,pxhat2_3=0,z0=0;
    double dvhat3_3,vhat3_3=0,dpxhat3_3,pxhat3_3=0,z1=0;
    double theta=2, lembda=.3; // Gains
    ROS_INFO("start");
    ros::init(argc, argv, "move_pub_agent3");
    ros::NodeHandle n;
    
    ros::Subscriber sub_odometryagent3 = n.subscribe("/agent3/odom", 10, odomCallbackagent3);
    ros::Subscriber sub_odometryagent2 = n.subscribe("/agent2/odom", 10, odomCallbackagent2);
    ros::Publisher movement_pubagent3 = n.advertise<geometry_msgs::Twist>("/agent3/cmd_vel",10); 
    //for sensors the value after , should be higher to get a more accurate result (queued)
    ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement
   //for sensors the value after , should be higher to get a more accurate result (queued)
    //move
    ros::Time start1 = ros::Time::now();
    while(ros::ok())
    {
	geometry_msgs::Twist move;
	ros::Time t = ros::Time::now();
   	// calculate delta_t
	delta_t = t - prev_time;
	prev_time = ros::Time::now();

        //Observer to estimate agent2's states
        z0=exp(-2*theta*(t.toSec()-tk2_3.toSec()))*(pxhat2_3-px_agent2);
        dvhat2_3=-theta*theta*z0;
	vhat2_3 +=dvhat2_3*delta_t.toSec();
	dpxhat2_3=vhat2_3-2*theta*z0;
	pxhat2_3 +=dpxhat2_3*delta_t.toSec();
        
	//Observer to estimate agent3's states
        z1=exp(-2*theta*(t.toSec()-tk3_3.toSec()))*(pxhat3_3-px_agent3);
        dvhat3_3=-theta*theta*z1;
	vhat3_3 +=dvhat3_3*delta_t.toSec();
	dpxhat3_3=vhat3_3-2*theta*z1;
	pxhat3_3 +=dpxhat3_3*delta_t.toSec();

	// calculating u
	u=-(lembda*lembda)*(pxhat3_3-pxhat2_3)-lembda*(vhat3_3-vhat2_3);
	u_integral += u*delta_t.toSec();
        
	//velocity controls
        move.linear.x = u_integral; //speed value m/s
        move.angular.z = 0;
        movement_pubagent3.publish(move);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
