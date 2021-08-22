#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "robot_wheel_speeds/DifferentialDriveEncoders.h"
#include "robot_wheel_speeds/SystemGpio.h"

int main(int argc, char *argv[])
{
    // Config
    int velocityUpdateIntervalNs = 2e7;
    ros::Duration velocityUpdateInterval(0, velocityUpdateIntervalNs);
    int leftPinA = 17;
    int leftPinB = 18;
    int rightPinA = 23;
    int rightPinB = 22;
    int ticksPerRevolution = 300;
    int wheelDiameterMm = 60;
    int wheelAxisMm = 70;

    ros::init(argc, argv, "robot_wheel_speeds");
    SystemGPIO gpio({leftPinA, leftPinB, rightPinA, rightPinB});

    ros::NodeHandle nodeHandle;

    auto publisher = nodeHandle.advertise<robot_wheel_speeds::WheelVelocities>(
        "wheel_speeds", 10);
    auto odom_pub = nodeHandle.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    // current position and angle
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Rate loop_rate(20);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ROS_INFO("Starting wheel_speeds");
    DifferentialDriveEncoders encoders(leftPinA, leftPinB, rightPinA, rightPinB,
                                       ticksPerRevolution, wheelDiameterMm, wheelAxisMm,
                                       velocityUpdateInterval);

    encoders.Start();
    ROS_INFO("Encoders started");
    int seq = 1;
    while (ros::ok())
    {
        // publish here
        auto msg = encoders.GetVelocities();
        msg.header.seq = seq++;

        publisher.publish(msg);

        /* START Odom tutorial */

        current_time = ros::Time::now();

        // Compute odometry
        double dt = (current_time - last_time).toSec();
        double vx = msg.linear.x;
        double vy = 0; // No strafing
        double vth = msg.angular.z;
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        /* END Odom tutorial */

        // check for incoming messages
        ros::spinOnce();

        last_time = current_time;
        loop_rate.sleep();
    }

    return 0;
}
