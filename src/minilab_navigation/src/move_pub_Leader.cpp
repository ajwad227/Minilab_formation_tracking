#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

geometry_msgs::Pose2D current_pose_leader;

void odomCallbackLeader(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose_leader.x = msg->pose.pose.position.x;
    current_pose_leader.y = msg->pose.pose.position.y;

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

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;
    double velocity;
    double u,u_integral = 0.0;
    ros::Time int_time,current_time,prev_time;
    ros::Duration delta_t;
    ROS_INFO("start");

    ros::init(argc, argv, "move_pub_Leader");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("/leader/odom", 10, odomCallbackLeader);
    ros::Publisher movement_pub = n.advertise<geometry_msgs::Twist>("/leader/cmd_vel",10); 
    //for sensors the value after , should be higher to get a more accurate result (queued)
    ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement

    //move forward 1
    ROS_INFO("move forward");
    ros::Time start1 = ros::Time::now();
    while(ros::ok() && current_pose_leader.x < 15)
    {
        geometry_msgs::Twist move;
	ros::Time t = ros::Time::now();
   	// calculate delta_t
	delta_t = t - prev_time;
	prev_time = ros::Time::now();
        //velocity controls
	//velocity=(sin((2*PI*ros::Time::now().toSec())/100));
	//if (velocity<0)
	//{
	//	velocity=-1*velocity;	
	//}
        //move.linear.x =sin((2*PI*ros::Time::now().toSec())/100);//0.1+ velocity; //speed value m/s
	move.linear.x =0.0;        
	move.angular.z = 0;
	//u=.01;
	//u_integral += u*delta_t.toSec();//velocity
        //move.linear.x =u_integral;//0.1+ velocity; //speed value m/s       
	//move.angular.z = 0;        
	movement_pub.publish(move);

        ros::spinOnce();
        rate.sleep();
    }

    // just stop
    while(ros::ok()) {
        geometry_msgs::Twist move;
        move.linear.x = 0;
        move.angular.z = 0;
        movement_pub.publish(move);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
