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

Eigen::Vector3f position(0,0,0);
Eigen::Vector3f positionEstimates(0,0,0);
Eigen::Vector3f velocity_IF(0,0,0);
Eigen::Vector3f velocityEstimates(0,0,0);

Eigen::Vector3f attitude(0,0,0);
Eigen::Vector3f attitudeEstimates(0,0,0);

Eigen::Vector3f linearDisturbanceEstimates(0,0,0);

Eigen::Vector3f posRef(0,0,0);
Eigen::Vector3f velRef(0,0,0);
Eigen::Vector3f attRef(0,0,0);
Eigen::Vector3f error_pos(0,0,0);
Eigen::Vector3f error_vel(0,0,0);

Eigen::Vector3f sigma(0,0,0);
Eigen::Vector3f xi_1(0,0,0);
Eigen::Vector3f xi_2(0,0,0);
Eigen::Vector3f lambda(0,0,0);
Eigen::Vector3f gam(0,0,0);

Eigen::Vector3f asmc(0,0,0);
Eigen::Vector3f k(0,0,0);
Eigen::Vector3f k_dot(0,0,0);
Eigen::Vector3f alpha(0,0,0);
Eigen::Vector3f beta(0,0,0);

float roll_des=0;
float pitch_des=0;
float yaw_des;
float step_size = 0.01;

float m = 2;
float thrust = 0;
float g = 9.81;
float z_ddot_des = 0;
float roll_des_arg = 0;
float pitch_des_arg = 0;

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

void posCallback(const geometry_msgs::Vector3::ConstPtr& p)
{
    position(0) = p->x;
    position(1) = p->y;
    position(2) = p->z;
}

void velIFCallback(const geometry_msgs::Vector3::ConstPtr& v)
{
    velocity_IF(0) = v->x;
    velocity_IF(1) = v->y;
    velocity_IF(2) = v->z;
}

void attCallback(const geometry_msgs::Vector3::ConstPtr& a)
{
    attitude(0) = a->x;
    attitude(1) = a->y;
    attitude(2) = a->z;
}

void posRefCallback(const geometry_msgs::Vector3::ConstPtr& pr)
{
    posRef(0) = pr->x;
    posRef(1) = pr->y;
    posRef(2) = pr->z;
}

void yawRefCallback(const std_msgs::Float64::ConstPtr& yD)
{
    yaw_des = yD->data;
}

void velRefCallback(const geometry_msgs::Quaternion::ConstPtr& vr)
{
    velRef(0) = vr->x;
    velRef(1) = vr->y;
    velRef(2) = vr->z;
    z_ddot_des = vr->w;
}

void posEstCallback(const geometry_msgs::Vector3::ConstPtr& pE)
{
    positionEstimates(0) = pE->x;
    positionEstimates(1) = pE->y;
    positionEstimates(2) = pE->z;
}

void velEstCallback(const geometry_msgs::Vector3::ConstPtr& vE)
{
    velocityEstimates(0) = vE->x;
    velocityEstimates(1) = vE->y;
    velocityEstimates(2) = vE->z;
}

void attEstCallback(const geometry_msgs::Vector3::ConstPtr& aE)
{
    attitudeEstimates(0) = aE->x;
    attitudeEstimates(1) = aE->y;
    attitudeEstimates(2) = aE->z;
}

void LinDistEstCallback(const geometry_msgs::Vector3::ConstPtr& ldE)
{
    linearDisturbanceEstimates(0) = ldE->x;
    linearDisturbanceEstimates(1) = ldE->y;
    linearDisturbanceEstimates(2) = ldE->z;
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pos_ctrl");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    ros::Subscriber pos_sub = nh.subscribe("quad_position", 100, &posCallback);
    ros::Subscriber pos_est_sub = nh.subscribe("position_estimates", 100, &posEstCallback);

    ros::Subscriber vel_IF_sub = nh.subscribe("quad_velocity_IF", 100, &velIFCallback);
    ros::Subscriber vel_est_sub = nh.subscribe("velocity_estimates", 100, &velEstCallback);
    
    ros::Subscriber att_sub = nh.subscribe("quad_attitude", 100, &attCallback);
    ros::Subscriber att_est_sub = nh.subscribe("attitude_estimates", 100, &attEstCallback);

    ros::Subscriber linear_dist_est_sub = nh.subscribe("disturbance_estimates", 100, &LinDistEstCallback);

    ros::Subscriber posRef_sub = nh.subscribe("desired_position", 100, &posRefCallback);
    ros::Subscriber velRef_sub = nh.subscribe("desired_velocity", 100, &velRefCallback);
    ros::Subscriber yawDes_sub = nh.subscribe("desired_yaw", 100, &yawRefCallback);

    ros::Publisher att_des_pub = nh.advertise<geometry_msgs::Vector3>("desired_attitude",100);    
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float64>("thrust",100);
    ros::Publisher k_pos_pub = nh.advertise<geometry_msgs::Vector3>("k_pos",100);
    ros::Publisher error_pos_pub = nh.advertise<geometry_msgs::Vector3>("error_pos",100);
    ros::Publisher error_vel_pub = nh.advertise<geometry_msgs::Vector3>("error_vel",100);
    ros::Publisher sigma_pos_pub = nh.advertise<geometry_msgs::Vector3>("sigma_pos",100);
    ros::Publisher asmc_pos_pub = nh.advertise<geometry_msgs::Vector3>("asmc_pos",100);

    ros::Publisher pitch_des_arg_pub = nh.advertise<std_msgs::Float64>("pitch_des_arg",100);
    ros::Publisher roll_des_arg_pub = nh.advertise<std_msgs::Float64>("roll_des_arg",100);

    geometry_msgs::Vector3 att_des_var;
    geometry_msgs::Vector3 k_pos_var;
    geometry_msgs::Vector3 error_pos_var;
    geometry_msgs::Vector3 error_vel_var;
    geometry_msgs::Vector3 sigma_pos_var;
    geometry_msgs::Vector3 asmc_pos_var;
    std_msgs::Float64 thrust_var;
    std_msgs::Float64 pitch_des_arg_var;
    std_msgs::Float64 roll_des_arg_var;

    
    xi_1 << 1, 1, 1;
    xi_2 << 6, 6, 1;
    gam << 1.3, 1.3, 1.5; // 1 < gamma < 2
    lambda << 1.5, 1.5, 2; // lambda > gamma    
    k << 0,0,0;
    k_dot << 0,0,0;
    alpha << 0.02, 0.02, 1;
    beta << 9, 9, 1;

    thrust_var.data = 0;
    thrust_pub.publish(thrust_var);   

    att_des_var.x = 0;
    att_des_var.y = 0;
    att_des_var.z = 0;
    att_des_pub.publish(att_des_var);
    ros::Duration(2).sleep();

    while(ros::ok())
    { 

        //Nonsingular terminal sliding surface
        for (int i = 0; i <= 2; i++)
        {
            //Error 
            error_pos(i) = positionEstimates(i) - posRef(i);
            error_vel(i)= velocityEstimates(i) - velRef(i);

            //error_pos(i) = position(i) - posRef(i);
            //error_vel(i)= velocity_IF(i) - velRef(i);

            sigma(i) = error_pos(i) + xi_1(i) * powf(std::abs(error_pos(i)),lambda(i)) * sign(error_pos(i)) + xi_2(i) * powf(std::abs(error_vel(i)),gam(i)) * sign(error_vel(i));
            
            k_dot(i) = powf(alpha(i),0.5) * powf(std::abs(sigma(i)),0.5) - powf(beta(i),0.5) * powf(k(i),2);

            k(i) = k(i) + k_dot(i) * step_size;

            asmc(i) = -2*k(i) * powf(std::abs(sigma(i)),0.5) * sign(sigma(i)) - (powf(k(i),2)/2) * sigma(i);
        }

        asmc(0) = asmc(0) - linearDisturbanceEstimates(0);
        asmc(1) = asmc(1) - linearDisturbanceEstimates(1);
        asmc(2) = asmc(2) - linearDisturbanceEstimates(2);
        //thrust = (m/(cos(attitudeEstimates(0))*cos(attitudeEstimates(1)))) * (asmc(2)  + 0 -9.81 - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_vel(2)), 2-gam(2))*sign(error_vel(2)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_vel(2)), 2-gam(2))*sign(error_vel(2))*xi_1(2)*lambda(2)*powf(std::abs(error_pos(2)), lambda(2)-1));
        thrust = (m/(cos(attitudeEstimates(0))*cos(attitudeEstimates(1)))) * (asmc(2) + 0 - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_vel(2)), 2-gam(2))*sign(error_vel(2)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_vel(2)), 2-gam(2))*sign(error_vel(2))*xi_1(2)*lambda(2)*powf(std::abs(error_pos(2)), lambda(2)-1));

        
        if (thrust > 0)
        {
            thrust = 0;
        }
        else if (thrust < -30)
        {
            thrust = -30;
        }

        if (thrust == 0 || position(2)==0)
        {
            roll_des = 0;
            pitch_des = 0;
        }
        else
        {
            roll_des_arg = (m/thrust)*(sin(yaw_des)*asmc(0) - cos(yaw_des)*asmc(1));
            if (roll_des_arg>1)
            {
                roll_des_arg = 1;
            }
            else if (roll_des_arg<-1)
            {
                roll_des_arg = -1;
            }

            roll_des = asin(roll_des_arg);

            pitch_des_arg = ((m/thrust)*asmc(0) - sin(yaw_des)*sin(roll_des)) / (cos(yaw_des)*cos(roll_des));
            if (pitch_des_arg>1)
            {
                pitch_des_arg = 1;
            }
            else if (pitch_des_arg<-1)
            {   
                pitch_des_arg = -1;
            }

            pitch_des = asin(pitch_des_arg);
        }
        
        

        

        att_des_var.x = roll_des;
        att_des_var.y = pitch_des;
        att_des_var.z = yaw_des;

        thrust_var.data = thrust;

        error_pos_var.x = error_pos(0);
        error_pos_var.y = error_pos(1);
        error_pos_var.z = error_pos(2);

        error_vel_var.x = error_vel(0);
        error_vel_var.y = error_vel(1);
        error_vel_var.z = error_vel(2);

        k_pos_var.x = k(0);
        k_pos_var.y = k(1);
        k_pos_var.z = k(2);

        sigma_pos_var.x = sigma(0);
        sigma_pos_var.y = sigma(1);
        sigma_pos_var.z = sigma(2);

        asmc_pos_var.x = asmc(0);
        asmc_pos_var.y = asmc(1);
        asmc_pos_var.z = asmc(2);

        pitch_des_arg_var.data = pitch_des_arg;
        roll_des_arg_var.data= roll_des_arg;


        att_des_pub.publish(att_des_var);
        thrust_pub.publish(thrust_var);
        k_pos_pub.publish(k_pos_var);
        error_pos_pub.publish(error_pos_var);
        error_vel_pub.publish(error_vel_var);
        sigma_pos_pub.publish(sigma_pos_var);
        asmc_pos_pub.publish(asmc_pos_var);
        pitch_des_arg_pub.publish(pitch_des_arg_var);
        roll_des_arg_pub.publish(roll_des_arg_var);


        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
