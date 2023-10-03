//Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Vector3f attitude_des;
Eigen::Vector3f attitude_vel_des;
Eigen::Vector3f attitude;
Eigen::Vector3f attitude_vel;

Eigen::Vector3f error;
Eigen::Vector3f error_dot;

////////////////////Sliding surface and ASMC///////////////////
Eigen::Vector3f ss;
Eigen::Vector3f xi_1;
Eigen::Vector3f lambda;
Eigen::Vector3f xi_2;
Eigen::Vector3f varpi;
Eigen::Vector3f vartheta;
Eigen::Vector3f asmc;
Eigen::Vector3f K1;
Eigen::Vector3f K1_dot;
Eigen::Vector3f K2;
Eigen::Vector3f k_reg;
Eigen::Vector3f kmin;
Eigen::Vector3f mu;

////////////////Outputs (Torques)////////////////////
Eigen::Vector3f tau; //tau_phi, //tau_theta //tau_psi

float step = 0.01;

///////////////Quad's parameters/////////////////////
float Jxx = 0.0411;
float Jyy = 0.0478;
float Jzz = 0.0599;

float yaw_ddot_des;
void attDesCallback(const geometry_msgs::Quaternion::ConstPtr& attD)
{
	attitude_des(0) = attD->x;
	attitude_des(1) = attD->y;
	attitude_des(2) = attD->z;
	attitude_vel_des(2) = attD->w;
}

void attCallback(const geometry_msgs::Vector3::ConstPtr& att)
{
	attitude(0) = att->x;
	attitude(1) = att->y;
	attitude(2) = att->z;
}

void attVelCallback(const geometry_msgs::Vector3::ConstPtr& attVel)
{
	attitude_vel(0) = attVel->x;
	attitude_vel(1) = attVel->y;
	attitude_vel(2) = attVel->z;
}

void yawddotVelCallback(const std_msgs::Float64::ConstPtr& ydd)
{
	yaw_ddot_des = ydd->data;
}

float sign(float value)
{
	int result;
	
	if(value > 0)
	{
		result = 1;
	}
	
	else if(value < 0)
	{
		result = -1;
	}
	
	else if(value == 0)
	{
		result = 0;
	}
	
	return result;
}

int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "attitude_nftasmc");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber desired_att_sub = nh.subscribe("desired_attitude",100, &attDesCallback);
	ros::Subscriber quad_attitude_sub = nh.subscribe("quad_attitude",100, &attCallback);
	ros::Subscriber quad_attitude_velocity_sub = nh.subscribe("quad_attitude_velocity",100, &attVelCallback);
	ros::Subscriber yaw_ddot_des_sub = nh.subscribe("yaw_ddot_desired",100, &yawddotVelCallback);
	
	
	geometry_msgs::Vector3 quadTorques;
	geometry_msgs::Vector3 adaptive_gains_att;
	geometry_msgs::Vector3 ss_att;	
	
	ros::Publisher quad_torques_pub = nh.advertise<geometry_msgs::Vector3>("quad_torques",100);
	ros::Publisher adaptive_gain_att_pub = nh.advertise<geometry_msgs::Vector3>("adaptive_gain_attitude",100);
	ros::Publisher sigma_att_pub = nh.advertise<geometry_msgs::Vector3>("sigma_att",100);
	
    xi_1 << 2, 2, 1; //1, 1, 1
    lambda << 1.5, 1.5, 1.5;
    xi_2 << 3, 3, 3;
    varpi << 4, 4, 4;
    vartheta << 3, 3, 3;
    K1 << 0, 0, 0;
    K2 << 0.01, 0.01, 0.1;
    k_reg << 1, 1, 1;
    kmin << 3, 3, 1;
    mu << 0.1, 0.1, 0.1;

	attitude_vel_des(0) = 0;
	attitude_vel_des(1) = 0;

	ros::Duration(2.1).sleep();
	while(ros::ok())
	{	
		for(int i = 0; i <= 2; i++)
		{	
			error(i) = attitude(i) - attitude_des(i);
			error_dot(i) = attitude_vel(i) - attitude_vel_des(i);

			ss(i) = error(i) + xi_1(i) * powf(std::abs(error(i)),lambda(i)) * sign(error(i)) + xi_2(i) * powf(std::abs(error_dot(i)),(varpi(i)/vartheta(i))) * sign(error_dot(i));
			
			if(K1(i) > kmin(i))	
			{
				K1_dot(i) = k_reg(i) * sign(std::abs(ss(i))-mu(i));
			}
			else
			{
				K1_dot(i) = kmin(i);
			}
			
			K1(i) = K1(i) + step * K1_dot(i); //New value of K1
			asmc(i) = -K1(i) * powf(std::abs(ss(i)),0.5) * sign(ss(i)) - K2(i) * ss(i);
		}
		
		tau(0) = Jxx * (asmc(0) - (((Jyy-Jzz)/Jxx) * attitude_vel(1) * attitude_vel(2)) - (vartheta(0)/(varpi(0)*xi_2(0))) * sign(error_dot(0)) * powf(std::abs(error_dot(0)),(2-(varpi(0)/vartheta(0)))) * (1 + xi_1(0) * lambda(0) * powf(std::abs(error(0)),lambda(0)-1)));
		tau(1) = Jyy * (asmc(1) - (((Jzz-Jxx)/Jyy) * attitude_vel(0) * attitude_vel(2)) - (vartheta(1)/(varpi(1)*xi_2(1))) * sign(error_dot(1)) * powf(std::abs(error_dot(1)),(2-(varpi(1)/vartheta(1)))) * (1 + xi_1(1) * lambda(1) * powf(std::abs(error(1)),lambda(1)-1)));
		tau(2) = Jzz * (asmc(2) - (((Jxx-Jyy)/Jzz) * attitude_vel(0) * attitude_vel(1)) - (vartheta(2)/(varpi(2)*xi_2(2))) * sign(error_dot(2)) * powf(std::abs(error_dot(2)),(2-(varpi(2)/vartheta(2)))) * (1 + xi_1(2) * lambda(2) * powf(std::abs(error(2)),lambda(2)-1)));


		quadTorques.x = tau(0);
		quadTorques.y = tau(1);
		quadTorques.z = tau(2);
		
		adaptive_gains_att.x = K1(0);
		adaptive_gains_att.y = K1(1);
		adaptive_gains_att.z = K1(2);
			
		ss_att.x = ss(0);
		ss_att.y = ss(1);
		ss_att.z = ss(2);	
			
		quad_torques_pub.publish(quadTorques);
		adaptive_gain_att_pub.publish(adaptive_gains_att);
		sigma_att_pub.publish(ss_att);				
			
		std::cout << "Error_yaw " << error(2) << std::endl;
		//std::cout << "Torque_pitch " << tau(1) << std::endl;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
