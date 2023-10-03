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

float step = 0.01;
float gravity = 9.81;
float quad_mass = 2;
float thrust = quad_mass*gravity;

Eigen::Vector3f linear_position;
Eigen::Vector3f attitude_position;

Eigen::Vector3f linear_velocity_BF;
Eigen::Vector3f linear_velocity_IF;
Eigen::Vector3f attitude_velocity;

Eigen::Vector3f linear_acceleration_BF;
Eigen::Vector3f attitude_acceleration;

Eigen::Vector3f tau;
Eigen::Vector3f dist;

Eigen::Vector3f force(0,0,0);

Eigen::Vector3f e3(0,0,1);

Eigen::Matrix3f J;

Eigen::Matrix3f RotationMatrix(Eigen::Vector3f attitude)
{
	float cos_phi = cos(attitude(0));
	float sin_phi = sin(attitude(0));
	
	float cos_theta = cos(attitude(1));
	float sin_theta = sin(attitude(1));
	
	
	float cos_psi = cos(attitude(2));
	float sin_psi = sin(attitude(2));
	
	
	Eigen::Matrix3f R;
	
	R << cos_psi * cos_theta, cos_psi * sin_phi * sin_theta - cos_phi * sin_psi, sin_psi * sin_phi + cos_psi * cos_phi * sin_theta,
			 cos_theta * sin_psi, cos_psi * cos_phi + sin_psi * sin_phi * sin_theta, cos_phi * sin_psi * sin_theta - cos_psi * sin_phi,
			 -sin_theta, cos_theta * sin_phi, cos_phi * cos_theta;
			 
	return R;
	
}

Eigen::Matrix3f skewOmega(Eigen::Vector3f attitude_velocity)
{
	Eigen::Matrix3f Skew;
	
	Skew << 0, -attitude_velocity(2), attitude_velocity(1),
					attitude_velocity(2), 0, -attitude_velocity(0),
					-attitude_velocity(1), attitude_velocity(0), 0;
					
	return Skew;

}

void ThrustInputCallback(const std_msgs::Float64::ConstPtr& thr)
{
	thrust = thr->data;
}

void TorqueInputsCallback(const geometry_msgs::Vector3::ConstPtr& tor)
{
	tau(0) = tor->x;
	tau(1) = tor->y;
	tau(2) = tor->z;
}

void DisturbancesCallback(const geometry_msgs::Vector3::ConstPtr& d)
{
	dist(0) = d->x;
	dist(1) = d->y;
	dist(2) = d->z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uav_dynamics");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber thrust_sub = nh.subscribe("quad_thrust",100,&ThrustInputCallback);
	ros::Subscriber quad_torques_sub = nh.subscribe("quad_torques",100,&TorqueInputsCallback);
	ros::Subscriber disturbances_sub = nh.subscribe("disturbances",100,&DisturbancesCallback);
	
	geometry_msgs::Vector3 positionVector;
	geometry_msgs::Vector3 attitudeVector;
	geometry_msgs::Vector3 velocityVector;
	geometry_msgs::Vector3 att_velVector;
	geometry_msgs::Vector3 BFvelocityVector;
	
	ros::Publisher positionPub = nh.advertise<geometry_msgs::Vector3>("quad_position",100);
	ros::Publisher quad_attitude_pub = nh.advertise<geometry_msgs::Vector3>("quad_attitude",100);
	ros::Publisher velocityPub = nh.advertise<geometry_msgs::Vector3>("quad_velocity",100);
	ros::Publisher quad_attitude_velocity_pub = nh.advertise<geometry_msgs::Vector3>("quad_attitude_velocity",100);
	ros::Publisher quad_vel_BF_pub = nh.advertise<geometry_msgs::Vector3>("quad_velocity_BF",100);
	
	
	J << 0.0411, 0, 0,
		 0, 0.0478, 0,
		 0, 0, 0.0599;
	
	//linear_position << -24.1,-3.2,-4; //Recta
	//linear_position << -0.1,-2.8,-4; //Circulo
	//linear_position << -4.7,-0.2,-4; //Lemniscata
	linear_position << 0.2,0.3,-4; // Target perturbado
	
	attitude_position << 0,0,0;

	linear_velocity_BF << 0,0,0;
	linear_velocity_IF << 0,0,0;
	attitude_velocity << 0,0,0;

	linear_acceleration_BF << 0,0,0;
	attitude_acceleration << 0,0,0;

		 
	positionVector.x = linear_position(0);
	positionVector.y = linear_position(1);
	positionVector.z = linear_position(2);
	
	attitudeVector.x = attitude_position(0);
	attitudeVector.y = attitude_position(1);
	attitudeVector.z = attitude_position(2);

	BFvelocityVector.x = linear_velocity_BF(0);
	BFvelocityVector.y = linear_velocity_BF(1);
	BFvelocityVector.z = linear_velocity_BF(2);
	
	positionPub.publish(positionVector);
	quad_attitude_pub.publish(attitudeVector);
	quad_vel_BF_pub.publish(BFvelocityVector);
	//ros::Duration(0.05).sleep();
	positionPub.publish(positionVector);
	quad_attitude_pub.publish(attitudeVector);
	quad_vel_BF_pub.publish(BFvelocityVector);
	ros::Duration(4).sleep();
	
	while(ros::ok())
	{
		//Angular dynamics
		attitude_acceleration = J.inverse() * (tau - (skewOmega(attitude_velocity) * J * attitude_velocity));
	
		for(int i = 0; i <= 2; i++)
		{
			attitude_velocity(i) = attitude_velocity(i) + step * attitude_acceleration(i);
			attitude_position(i) = attitude_position(i) + step * attitude_velocity(i);
		}
		
		//Linear dynamics
		force = (RotationMatrix(attitude_position).inverse() * (quad_mass * gravity * e3)) - (thrust * e3) + RotationMatrix(attitude_position).inverse() * dist; 
		
		linear_acceleration_BF = force/quad_mass - skewOmega(attitude_velocity) * linear_velocity_BF; 
		 
		for(int i = 0; i <= 2; i++)
		{
			linear_velocity_BF(i) = linear_velocity_BF(i) + step * linear_acceleration_BF(i);
		}
		
		linear_velocity_IF = RotationMatrix(attitude_position) * linear_velocity_BF;
		
		for(int i = 0; i <= 2; i++)
		{
			linear_position(i) = linear_position(i) + step * linear_velocity_IF(i);
		}
		
		positionVector.x = linear_position(0);
		positionVector.y = linear_position(1);
		positionVector.z = linear_position(2);
		
		attitudeVector.x = attitude_position(0);
		attitudeVector.y = attitude_position(1);
		attitudeVector.z = attitude_position(2);
		
		velocityVector.x = linear_velocity_IF(0);
		velocityVector.y = linear_velocity_IF(1);
		velocityVector.z = linear_velocity_IF(2);
		
		att_velVector.x = attitude_velocity(0);
		att_velVector.y = attitude_velocity(1);
		att_velVector.z = attitude_velocity(2);
		
		BFvelocityVector.x = linear_velocity_BF(0);
		BFvelocityVector.y = linear_velocity_BF(1);
		BFvelocityVector.z = linear_velocity_BF(2);
		
		positionPub.publish(positionVector);
		quad_attitude_pub.publish(attitudeVector);
		velocityPub.publish(velocityVector);
		quad_attitude_velocity_pub.publish(att_velVector);
		quad_vel_BF_pub.publish(BFvelocityVector);
		
		//std::cout << "U1 : " << U1 << std::endl;
		std::cout << "force: " << force(0) << std::endl; 
		
		ros::spinOnce();
		loop_rate.sleep();
				
	}
	return 0;	
}
