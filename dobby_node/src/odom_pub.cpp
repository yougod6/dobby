#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense>
#include "std_msgs/msg/int64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <chrono>
#include <cmath>

#define PI 3.14159265359

using std::placeholders::_1;
using namespace std::chrono_literals;


double xpos = 0.0;
double ypos = 0.0;
double theta = 0.0;

Eigen::Vector2d WheelSpeed;
Eigen::Vector3d ChassisSpeed;
Eigen::MatrixXd H(3,2);

double dt = 0.01;

// double delta_pi_l = 0.0;
// double delta_pi_r = 0.0;
double q[4] = {0.0};

double V_x = 0.0;
double V_y = 0.0;
double W = 0.0;

void quaternion_from_euler(double roll, double pitch, double yaw, double q[]);


class Odom : public rclcpp::Node
{
  public:
    Odom() : Node("odometry_node")
    {
      rpm_sub = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/rpm", 10, std::bind(&Odom::odometry_cal, this, _1));

      publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

      cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Odom::cmd_vel_callback, this, _1));
    }

  private:
    void odometry_cal(const std_msgs::msg::Int64MultiArray::SharedPtr msg) const
    {
      nav_msgs::msg::Odometry odom_msg; 
      
      double rpm_l = msg->data[0];
      double rpm_r = msg->data[1];

      double omega_l = (rpm_l * (2 * PI) / 60);
      double omega_r = (rpm_r * (2 * PI) / 60);

      // WheelSpeed <<omega_l, omega_r;
    

      // H <<      -r/(2*b)     ,     r/(2*b)     ,
      //       (r/2)*cos(theta) , (r/2)*cos(theta),
      //       (r/2)*sin(theta) , (r/2)*sin(theta);
        
      // ChassisSpeed = H * WheelSpeed;

      // double x_dot = ChassisSpeed(1);
      // double y_dot = ChassisSpeed(2);
      // double theta_dot = ChassisSpeed(0);
      double x_dot = (r/2)*(omega_l + omega_r)*cos(theta);
      double y_dot = (r/2)*(omega_l + omega_r)*sin(theta);
      double theta_dot = r/(2*b)*(omega_r - omega_l);

      xpos = xpos + x_dot * dt;
      ypos = ypos + y_dot * dt;
      theta = theta + theta_dot * dt;
      // double delta_pi_l = vel_l*dt;
      // double delta_pi_r = vel_r*dt;
      // xpos = xpos + (r/2)*(delta_pi_l + delta_pi_r)*cos(theta);
      // ypos = ypos + (r/2)*(delta_pi_l + delta_pi_r)*sin(theta);
      // theta = theta + (r/(2*b))*(delta_pi_r - delta_pi_l);

      
      quaternion_from_euler(0,0,theta,q);

      std::cout << "test"  << std::endl << "x_dot = " <<x_dot<< " " << "y_dot = " << y_dot << " " <<  "theta_dot = " <<theta_dot <<  std::endl;
      odom_msg.pose.pose.position.x = xpos;
      odom_msg.pose.pose.position.y = ypos;
      odom_msg.pose.pose.position.z = 0;
      odom_msg.pose.pose.orientation.x = q[1];
      odom_msg.pose.pose.orientation.y = q[2];
      odom_msg.pose.pose.orientation.z = q[3];
      odom_msg.pose.pose.orientation.w = q[0];
      odom_msg.twist.twist.linear.x = V_x;
      odom_msg.twist.twist.linear.y = V_y;
      odom_msg.twist.twist.linear.z = 0;
      odom_msg.twist.twist.angular.x = 0;
      odom_msg.twist.twist.angular.y = 0;
      odom_msg.twist.twist.angular.z = W;

      publisher->publish(odom_msg);

    }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      V_x = msg->linear.x;
      V_y = msg->linear.y;
      W = msg->angular.z;


    }

    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr rpm_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    double r = 0.04225; //Wheel radius 0.0845
    double b = 0.09255; //From robot center to wheel distance  0.1851
    
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odom>());
  rclcpp::shutdown();
  return 0;
}

void quaternion_from_euler(double roll, double pitch, double yaw, double q[])
{
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = sy * cp * sr + cy * sp * cr;
  q[3] = sy * cp * cr - cy * sp * sr;

}