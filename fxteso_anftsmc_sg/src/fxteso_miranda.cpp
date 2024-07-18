//Including ROS libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Vector3f position(0,0,0);
Eigen::Vector3f attitude(0,0,0);
Eigen::Vector3f velocity(0,0,0);
Eigen::Vector3f attVelocity(0,0,0);
float thrust = 0;
Eigen::Vector3f torques(0,0,0);

Eigen::VectorXf G1(6);
Eigen::VectorXf G2(6);
Eigen::VectorXf G3(6);
Eigen::VectorXf G4(6);

Eigen::VectorXf lambda(6);  // 0 < lambda < 1
Eigen::VectorXf varphi(6);  // varphi > 1 

Eigen::Vector3f pos_est(0,0,0);
Eigen::Vector3f vel_est(0,0,0);
Eigen::Vector3f dist_est(0,0,0);

Eigen::Vector3f att_est(0,0,0);
Eigen::Vector3f attvel_est(0,0,0);
Eigen::Vector3f att_dist_est(0,0,0);

Eigen::VectorXf x1_dot(6);
Eigen::VectorXf x2_dot(6);
Eigen::VectorXf x3_dot(6);

float fx = 0;
float gx_u = 0;
float step = 0.01;
float quad_mass = 2;
float jx = 0.0411;
float jy = 0.0478;
float jz = 0.0599;

Eigen::Vector3f estimation_error_linear(0,0,0);
Eigen::Vector3f estimation_error_angular(0,0,0);

Eigen::Vector3f distIF(0,0,0);

//MATH FUNCTIONS
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



//CALLBACKS
void posCallback(const geometry_msgs::Vector3::ConstPtr& p)
{
    position(0) = p->x;
    position(1) = p->y;
    position(2) = p->z;
}

void attCallback(const geometry_msgs::Vector3::ConstPtr& a)
{
    attitude(0) = a->x;
    attitude(1) = a->y;
    attitude(2) = a->z;
}

void velCallback(const geometry_msgs::Vector3::ConstPtr& v)
{
    velocity(0) = v->x;
    velocity(1) = v->y;
    velocity(2) = v->z;
}

void avelCallback(const geometry_msgs::Vector3::ConstPtr& av)
{
    attVelocity(0) = av->x;
    attVelocity(1) = av->y;
    attVelocity(2) = av->z;
}

void thrustCallback(const std_msgs::Float64::ConstPtr& th)
{
    thrust = th->data;
}

void torquesCallback(const geometry_msgs::Vector3::ConstPtr& to)
{
    torques(0) = to->x;
    torques(1) = to->y;
    torques(2) = to->z;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "fxteso_miranda");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    //Subscribers and publishers
    ros::Subscriber quav_pos_sub = nh.subscribe("quad_position", 100, &posCallback);
    ros::Subscriber quav_att_sub = nh.subscribe("quad_attitude", 100, &attCallback);

    ros::Subscriber quav_vel_sub = nh.subscribe("quad_velocity_IF", 100, &velCallback);
    ros::Subscriber quav_attVel_sub = nh.subscribe("quad_attitude_velocity", 100, &avelCallback);

    ros::Subscriber thrust_sub = nh.subscribe("thrust", 100, &thrustCallback);
    ros::Subscriber torques_sub = nh.subscribe("torques", 100, &torquesCallback);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Publisher positionEstimates_pub = nh.advertise<geometry_msgs::Vector3>("position_estimates",100);    
    ros::Publisher velocityEstimates_pub = nh.advertise<geometry_msgs::Vector3>("velocity_estimates",100); 
    ros::Publisher disturbanceEstimates_pub = nh.advertise<geometry_msgs::Vector3>("disturbance_estimates",100); 
    ros::Publisher dist_if_pub = nh.advertise<geometry_msgs::Vector3>("dists_if",100); 

    ros::Publisher attitudeEstimates_pub = nh.advertise<geometry_msgs::Vector3>("attitude_estimates",100); 
    ros::Publisher attVelEstimates_pub = nh.advertise<geometry_msgs::Vector3>("attVel_estimates",100); 
    ros::Publisher disturbanceAngEstimates_pub = nh.advertise<geometry_msgs::Vector3>("disturbance_angular_estimates",100); 

    ros::Publisher estimationError_linear_pub = nh.advertise<geometry_msgs::Vector3>("estimation_error_linear",100);
    ros::Publisher estimationError_angular_pub = nh.advertise<geometry_msgs::Vector3>("estimation_error_angular",100);
    


    geometry_msgs::Vector3 positionEstimates_var;
    geometry_msgs::Vector3 velocityEstimates_var;
    geometry_msgs::Vector3 disturbanceEstimates_var;
    geometry_msgs::Vector3 disturbanceIF_var;

    geometry_msgs::Vector3 attitudeEstimates_var;
    geometry_msgs::Vector3 attVelEstimates_var;
    geometry_msgs::Vector3 disturbanceAngEstimates_var;

    geometry_msgs::Vector3 estimationError_linear_var;
    geometry_msgs::Vector3 estimationError_angular_var;

    
    //FXTESO GAINS
    G1 << 16, 16, 16, 16, 16, 16;
    G2 << 150, 150, 150, 150, 150, 150;
    G3 << 450, 450, 450, 450, 450, 450;
    G4 << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

    lambda << 0.6, 0.6, 0.6, 0.6, 0.6, 0.6;
    varphi << 1.2, 1.2, 1.2, 1.2, 1.2, 1.2;


    //Initial conditions

    pos_est << 0.05,-0.05,-0.1;
    vel_est << 0,0,0;
    dist_est << 0,0,0;

    att_est << 0,0,0;
    attvel_est << 0,0,0;
    att_dist_est << 0,0,0;

    x1_dot << 0,0,0,0,0,0;
    x2_dot << 0,0,0,0,0,0;
    x3_dot << 0,0,0,0,0,0;

    estimation_error_linear(0) = position(0) - pos_est(0);
    estimation_error_linear(1) = position(1) - pos_est(1);
    estimation_error_linear(2) = position(2) - pos_est(2);

    estimation_error_angular(0) = attitude(0) - att_est(0);
    estimation_error_angular(1) = attitude(1) - att_est(1);
    estimation_error_angular(2) = attitude(2) - att_est(2);



    positionEstimates_var.x = pos_est(0);
    positionEstimates_var.y = pos_est(1);
    positionEstimates_var.z = pos_est(2);

    velocityEstimates_var.x = vel_est(0);
    velocityEstimates_var.y = vel_est(1);
    velocityEstimates_var.z = vel_est(2);

    disturbanceEstimates_var.x = dist_est(0);
    disturbanceEstimates_var.y = dist_est(1);
    disturbanceEstimates_var.z = dist_est(2);

    attitudeEstimates_var.x = att_est(0);
    attitudeEstimates_var.y = att_est(1);
    attitudeEstimates_var.z = att_est(2);

    attVelEstimates_var.x = attvel_est(0);
    attVelEstimates_var.y = attvel_est(1);
    attVelEstimates_var.z = attvel_est(2);

    estimationError_linear_var.x = estimation_error_linear(0);
    estimationError_linear_var.y = estimation_error_linear(1);
    estimationError_linear_var.z = estimation_error_linear(2);
    
    estimationError_angular_var.x = estimation_error_angular(0);
    estimationError_angular_var.y = estimation_error_angular(1);
    estimationError_angular_var.z = estimation_error_angular(2);

    disturbanceIF_var.x = 0;
    disturbanceIF_var.y = 0;
    disturbanceIF_var.z = 0;

    disturbanceAngEstimates_var.x = 0;
    disturbanceAngEstimates_var.y = 0;
    disturbanceAngEstimates_var.z = 0;


    positionEstimates_pub.publish(positionEstimates_var);
    velocityEstimates_pub.publish(velocityEstimates_var);
    disturbanceEstimates_pub.publish(disturbanceEstimates_var);
    attitudeEstimates_pub.publish(attitudeEstimates_var);
    attVelEstimates_pub.publish(attVelEstimates_var);
    estimationError_linear_pub.publish(estimationError_linear_var);
    estimationError_angular_pub.publish(estimationError_angular_var);
    dist_if_pub.publish(disturbanceIF_var);
    disturbanceAngEstimates_pub.publish(disturbanceAngEstimates_var);

    ros::Duration(2.68).sleep();
    //ros::Duration(3.7).sleep();

    while(ros::ok())
    { 
        ///////////////////////////////POSITION SUBSYSTEM///////////////////////////////////////////////////
        for(int i = 0; i<=2; i++)
        {
            //Estimation error
            estimation_error_linear(i) = position(i) - pos_est(i);
            
            x3_dot(i) = G3(i) * sign(estimation_error_linear(i)) * ( powf(std::abs(estimation_error_linear(i)),lambda(i))  +  powf(std::abs(estimation_error_linear(i)),varphi(i)) ) + G4(i) * sign(estimation_error_linear(i));
            dist_est(i) = dist_est(i) + x3_dot(i) * step;

            //Defining f(x) and g(x)u
            if(i == 0)
            {
                fx = 0;
                gx_u = (thrust/quad_mass) * (sin(att_est(0))*sin(att_est(2)) + cos(att_est(0)) * cos(att_est(2)) * sin(att_est(1)));
            }
            else if (i == 1)
            {
                fx = 0;
                gx_u = (thrust/quad_mass) * (cos(att_est(0))*sin(att_est(2))*sin(att_est(1)) - cos(att_est(2)) * sin(att_est(0)));                
            }
            else if (i == 2)
            {
                //fx = 9.81;
                fx = 0;
                gx_u = (thrust/quad_mass) * cos(att_est(0)) * cos(att_est(1));
            }

            // Proceding with the estimator
            x2_dot(i) = dist_est(i) + fx + gx_u + G2(i) * sign(estimation_error_linear(i)) * ( powf(std::abs(estimation_error_linear(i)),((lambda(i) + 1)/2))  +  powf(std::abs(estimation_error_linear(i)),(varphi(i) + 1)/2) );
            vel_est(i) = vel_est(i) + x2_dot(i) * step;

            x1_dot(i) = vel_est(i) + G1(i) * sign(estimation_error_linear(i)) * ( powf(std::abs(estimation_error_linear(i)),(lambda(i) + 2)/3)  +  powf(std::abs(estimation_error_linear(i)),(varphi(i) + 2)/3) );
            pos_est(i) = pos_est(i) + x1_dot(i) * step;

        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////ATTITUDE SUBSYSTEM////////////////////////////////////////////////////
        for(int i = 0; i<=2; i++)
        {
            //Estimation error
            estimation_error_angular(i) = attitude(i) - att_est(i);
            
            x3_dot(3+i) = G3(3+i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),lambda(3+i))  +  powf(std::abs(estimation_error_angular(i)),varphi(3+i)) ) + G4(3+i) * sign(estimation_error_angular(i));
            att_dist_est(i) = att_dist_est(i) + x3_dot(3+i) * step;

            //Defining f(x) and g(x)u
            if(i == 0) //Roll
            {
                //fx = ((jy-jz)/jx) * attVelocity(1) * attVelocity(2);
                fx = 0;
                gx_u = torques(0)/jx;
            }
            else if (i == 1) //Pitch
            {
                //fx = ((jz-jx)/jy) * attVelocity(0) * attVelocity(2);
                fx = 0;
                gx_u = torques(1)/jy;
            }
            else //Yaw
            {
                //fx = ((jx-jy)/jz) * attVelocity(0) * attVelocity(1);
                fx = 0;
                gx_u = torques(2)/jz;
            }

            // Proceding with the estimator
            x2_dot(3+i) = att_dist_est(i) + fx + gx_u + G2(3+i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),((lambda(3+i) + 1)/2))  +  powf(std::abs(estimation_error_angular(i)),(varphi(3+i) + 1)/2) );
            attvel_est(i) = attvel_est(i) + x2_dot(3+i) * step;

            x1_dot(3+i) = attvel_est(i) + G1(3+i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),(lambda(3+i) + 2)/3)  +  powf(std::abs(estimation_error_angular(i)),(varphi(3+i) + 2)/3) );
            att_est(i) = att_est(i) + x1_dot(3+i) * step;

        }

        positionEstimates_var.x = pos_est(0);
        positionEstimates_var.y = pos_est(1);
        positionEstimates_var.z = pos_est(2);

        velocityEstimates_var.x = vel_est(0);
        velocityEstimates_var.y = vel_est(1);
        velocityEstimates_var.z = vel_est(2);

        disturbanceEstimates_var.x = dist_est(0);
        disturbanceEstimates_var.y = dist_est(1);
        disturbanceEstimates_var.z = dist_est(2);

        attitudeEstimates_var.x = att_est(0);
        attitudeEstimates_var.y = att_est(1);
        attitudeEstimates_var.z = att_est(2);

        attVelEstimates_var.x = attvel_est(0);
        attVelEstimates_var.y = attvel_est(1);
        attVelEstimates_var.z = attvel_est(2);

        disturbanceAngEstimates_var.x = att_dist_est(0);
        disturbanceAngEstimates_var.y = att_dist_est(1);
        disturbanceAngEstimates_var.z = att_dist_est(2);

        estimationError_linear_var.x = estimation_error_linear(0);
        estimationError_linear_var.y = estimation_error_linear(1);
        estimationError_linear_var.z = estimation_error_linear(2);
    
        estimationError_angular_var.x = estimation_error_angular(0);
        estimationError_angular_var.y = estimation_error_angular(1);
        estimationError_angular_var.z = estimation_error_angular(2);

        disturbanceIF_var.x = 2*dist_est(0);
        disturbanceIF_var.y = 2*dist_est(1);
        disturbanceIF_var.z = 2*dist_est(2);

        positionEstimates_pub.publish(positionEstimates_var);
        velocityEstimates_pub.publish(velocityEstimates_var);
        disturbanceEstimates_pub.publish(disturbanceEstimates_var);
        attitudeEstimates_pub.publish(attitudeEstimates_var);
        attVelEstimates_pub.publish(attVelEstimates_var);
        estimationError_linear_pub.publish(estimationError_linear_var);
        estimationError_angular_pub.publish(estimationError_angular_var);

        dist_if_pub.publish(disturbanceIF_var);

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}