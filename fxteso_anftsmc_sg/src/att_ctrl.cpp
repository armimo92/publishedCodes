//Including ROS libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Vector3f attVelRef(0,0,0);
Eigen::Vector3f attRef(0,0,0);
Eigen::Vector3f error_att(0,0,0);
Eigen::Vector3f error_attVel(0,0,0);

Eigen::Vector3f attitude(0,0,0);
Eigen::Vector3f attitudeEstimates(0,0,0);

Eigen::Vector3f attitude_vel(0,0,0);
Eigen::Vector3f attitudeVelEstimates(0,0,0);

Eigen::Vector3f angularDisturbanceEstimates(0,0,0);

Eigen::Vector3f sigma(0,0,0);
Eigen::Vector3f xi_1(0,0,0);
Eigen::Vector3f xi_2(0,0,0);
Eigen::Vector3f gam(0,0,0);
Eigen::Vector3f lambda(0,0,0);

Eigen::Vector3f asmc(0,0,0);
Eigen::Vector3f k(0,0,0);
Eigen::Vector3f k_dot(0,0,0);
Eigen::Vector3f alpha(0,0,0);
Eigen::Vector3f beta(0,0,0);

Eigen::Vector3f tau;

float step_size = 0.01;

float Jxx = 0.0411;
float Jyy = 0.0478;
float Jzz = 0.0599;


float sign(float var)
{   
    float x;
    if (var > 0)
    {
        x = 1;
    }
    else if (var<0)
    {
        x = -1;
    }
    else if (var == 0)
    {
        x = 0;
    }
    return x;
}

void attDesCallback(const geometry_msgs::Vector3::ConstPtr& aD)
{
    attRef(0) = aD->x;
    attRef(1) = aD->y;
    attRef(2) = aD->z;
}

void attVelDesCallback(const geometry_msgs::Vector3::ConstPtr& avD)
{
    attVelRef(0) = avD->x;
    attVelRef(1) = avD->y;
    attVelRef(2) = avD->z;
}

void attCallback(const geometry_msgs::Vector3::ConstPtr& a)
{
    attitude(0) = a->x;
    attitude(1) = a->y;
    attitude(2) = a->z;
}

void attVelCallback(const geometry_msgs::Vector3::ConstPtr& av)
{
    attitude_vel(0) = av->x;
    attitude_vel(1) = av->y;
    attitude_vel(2) = av->z;
}

void attEstCallback(const geometry_msgs::Vector3::ConstPtr& aE)
{
    attitudeEstimates(0) = aE->x;
    attitudeEstimates(1) = aE->y;
    attitudeEstimates(2) = aE->z;
}

void attVelEstCallback(const geometry_msgs::Vector3::ConstPtr& avE)
{
    attitudeVelEstimates(0) = avE->x;
    attitudeVelEstimates(1) = avE->y;
    attitudeVelEstimates(2) = avE->z;
}

void angDistEstCallback(const geometry_msgs::Vector3::ConstPtr& adE)
{
    angularDisturbanceEstimates(0) = adE->x;
    angularDisturbanceEstimates(1) = adE->y;
    angularDisturbanceEstimates(2) = adE->z;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "att_ctrl");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    ros::Subscriber att_des_sub = nh.subscribe("desired_attitude", 100, &attDesCallback);
    ros::Subscriber attVel_des_sub = nh.subscribe("desired_attitude_velocity", 100, &attVelDesCallback);
    
    ros::Subscriber att_sub = nh.subscribe("quad_attitude", 100, &attCallback);
    ros::Subscriber att_est_sub = nh.subscribe("attitude_estimates", 100, &attEstCallback);

    ros::Subscriber attVel_est_sub = nh.subscribe("attVel_estimates", 100, &attVelEstCallback);
    ros::Subscriber attVel_sub = nh.subscribe("quad_attitude_velocity", 100, &attVelCallback);

    ros::Subscriber angular_dist_est_sub = nh.subscribe("disturbance_angular_estimates", 100, &angDistEstCallback);
   
    ros::Publisher torque_pub = nh.advertise<geometry_msgs::Vector3>("torques",100);
    ros::Publisher sigma_att_pub = nh.advertise<geometry_msgs::Vector3>("sigma_att",100);
    ros::Publisher error_att_pub = nh.advertise<geometry_msgs::Vector3>("error_att",100);
    ros::Publisher error_attVel_pub = nh.advertise<geometry_msgs::Vector3>("error_attVel",100);
    ros::Publisher k_att_pub = nh.advertise<geometry_msgs::Vector3>("k_att",100);

    geometry_msgs::Vector3 torque_var;
    geometry_msgs::Vector3 sigma_att_var;
    geometry_msgs::Vector3 error_att_var;
    geometry_msgs::Vector3 error_attVel_var;
    geometry_msgs::Vector3 k_att_var;

    xi_1 << 3, 3, 1;
    xi_2 << 1, 1, 1;
    lambda << 1.5, 1.5, 1.5;
    gam << 1.3, 1.3, 1.3;
    k << 0,0,0;
    k_dot << 0,0,0;
    alpha << 16, 16, 1;
    beta << 9, 9, 1;

    
    torque_var.x = 0;
    torque_var.y = 0;
    torque_var.z = 0;

    sigma_att_var.x = 0;
    sigma_att_var.y = 0;
    sigma_att_var.z = 0;

    error_att_var.x = 0;
    error_att_var.y = 0;
    error_att_var.z = 0;

    error_attVel_var.x = 0;
    error_attVel_var.y = 0;
    error_attVel_var.z = 0;
        
    k_att_var.x = k(0);
    k_att_var.y = k(1);
    k_att_var.z = k(2);
        
    k_att_pub.publish(k_att_var);
    torque_pub.publish(torque_var);
    sigma_att_pub.publish(sigma_att_var);
    error_att_pub.publish(error_att_var); 
    error_attVel_pub.publish(error_attVel_var);   

    ros::Duration(2).sleep();

    while(ros::ok())
    { 
        //Error 
        error_att = attitudeEstimates - attRef;
        error_attVel= attitudeVelEstimates - attVelRef;

        //error_att = attitude - attRef;
        //error_attVel= attitude_vel - attVelRef;

        //Nonsingular terminal sliding surface
        for (int i = 0; i <= 2; i++)
        {
            sigma(i) = error_att(i) + xi_1(i) * powf(std::abs(error_att(i)),lambda(i)) * sign(error_att(i)) + xi_2(i) * powf(std::abs(error_attVel(i)),(gam(i))) * sign(error_attVel(i));

            k_dot(i) = powf(alpha(i),0.5) * powf(std::abs(sigma(i)),0.5) - powf(beta(i),0.5) * powf(k(i),2);

            k(i) = k(i) + k_dot(i) * step_size;

            asmc(i) = -2*k(i) * powf(std::abs(sigma(i)),0.5) * sign(sigma(i)) - (powf(k(i),2)/2) * sigma(i);
        }

        asmc(0) = asmc(0) - angularDisturbanceEstimates(0);
        asmc(1) = asmc(1) - angularDisturbanceEstimates(1);
        asmc(2) = asmc(2) - angularDisturbanceEstimates(2);




        tau(0) = Jxx * ( - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0)) - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0))*xi_1(0)*lambda(0)*powf(std::abs(error_att(0)), lambda(0)-1) + asmc(0) );
        tau(1) = Jyy * ( - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1)) - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1))*xi_1(1)*lambda(1)*powf(std::abs(error_att(1)), lambda(1)-1) + asmc(1) );
        tau(2) = Jzz * ( - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2))*xi_1(2)*lambda(2)*powf(std::abs(error_att(2)), lambda(2)-1) + asmc(2) );


        //tau(0) = Jxx * ( -(((Jyy-Jzz)/Jxx)*attitude_vel(1)*attitude_vel(2)) - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0)) - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0))*xi_1(0)*lambda(0)*powf(std::abs(error_att(0)), lambda(0)-1) + asmc(0) );
        //tau(1) = Jyy * ( -(((Jzz-Jxx)/Jyy)*attitude_vel(0)*attitude_vel(2)) - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1)) - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1))*xi_1(1)*lambda(1)*powf(std::abs(error_att(1)), lambda(1)-1) + asmc(1) );
        //tau(2) = Jzz * ( -(((Jxx-Jyy)/Jzz)*attitude_vel(0)*attitude_vel(1)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2))*xi_1(0)*lambda(2)*powf(std::abs(error_att(2)), lambda(2)-1) + asmc(2) );

        torque_var.x = tau(0);
        torque_var.y = tau(1);
        torque_var.z = tau(2);

        sigma_att_var.x = sigma(0);
        sigma_att_var.y = sigma(1);
        sigma_att_var.z = sigma(2);

        error_att_var.x = error_att(0);
        error_att_var.y = error_att(1);
        error_att_var.z = error_att(2);

        error_attVel_var.x = error_attVel(0);
        error_attVel_var.y = error_attVel(1);
        error_attVel_var.z = error_attVel(2);
        
        k_att_var.x = k(0);
        k_att_var.y = k(1);
        k_att_var.z = k(2);
        
        k_att_pub.publish(k_att_var);
        torque_pub.publish(torque_var);
        sigma_att_pub.publish(sigma_att_var);
        error_att_pub.publish(error_att_var);
        error_attVel_pub.publish(error_attVel_var);
       
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
