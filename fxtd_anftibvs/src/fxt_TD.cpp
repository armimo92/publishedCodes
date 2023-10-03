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

float step = 0.02;

Eigen::Vector4f imFeat; //qx, qy, qz, qpsi
Eigen::Vector4f imFeat_estimate(0,0,1.7,0);
Eigen::Vector4f imFeat_estimate_dot(0,0,0,0);
Eigen::Vector4f x1_hat(0,0,0,0);
Eigen::Vector4f x2_hat(0,0,0,0);
Eigen::Vector4f x3_hat(0,0,0,0);
Eigen::Vector4f x4_hat(0,0,0,0);
Eigen::Vector4f x1_hat_dot(0,0,0,0);
Eigen::Vector4f x2_hat_dot(0,0,0,0);
Eigen::Vector4f new_x1_hat_dot(0,0,0,0);
Eigen::Vector4f new_x2_hat_dot(0,0,0,0);
Eigen::Vector4f x3_hat_dot(0,0,0,0);
Eigen::Vector4f x4_hat_dot(0,0,0,0);
Eigen::Vector4f G1;
Eigen::Vector4f G2;
Eigen::Vector4f G3;
Eigen::Vector4f k1;
Eigen::Vector4f k2;
Eigen::Vector4f k3;
Eigen::Vector4f k4;

Eigen::Vector4f estimation_error(0,0,0,0);
Eigen::Vector4f estimation_error2(0,0,0,0);

float alpha_1, alpha_2, beta_1, beta_2, alpha_3, beta_3;
float gamma_1, gamma_2, gamma_3, gamma_4;
float delta_1, delta_2, delta_3, delta_4;


float sign(float var)
{
    float result;
    if(var>0)
    {
        result = 1;
    }
    else if(var<0)
    {
        result = -1;
    }
    else if (var == 0)
    {
        result = 0;
    }
    return result;
}

float sig(float a, float b) //sig = |a|^b * sign(a)
{
    float s;
    s = powf(std::abs(a), b) * sign(a);
    return s;
}

float fal(float var, float pot, float delta)
{
    float res;
    if (std::abs(var) > delta)
    {
        res = powf(std::abs(var), pot) * sign(var);
    }
    if (std::abs(var) <= delta)
    {
        res = var/powf(delta, (1-pot));
    }

    return res;

}

void imFeatCallback(const geometry_msgs::Quaternion::ConstPtr& img_features)
{
	imFeat(0) = img_features->x;
	imFeat(1) = img_features->y;
	imFeat(2) = img_features->z;
    imFeat(3) = img_features->w;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fxt_TD");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);	

    //ROS publishers and subscribers
    ros::Subscriber im_feat_sub = nh.subscribe("ImFeat_vector", 100, &imFeatCallback);  


    ros::Publisher im_feat_est_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_estimates_fxt",100);
    ros::Publisher im_feat_est_err_pub = nh.advertise<geometry_msgs::Quaternion>("estimation_error_fxt",100);
    ros::Publisher im_feat_est_dot_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_dot_estimates_fxt",100);
    ros::Publisher im_feat_est_dot_pre_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_dot_estimates_fxt_pre",100);

    geometry_msgs::Quaternion im_feat_est_var;
    geometry_msgs::Quaternion im_feat_est_dot_var;
    geometry_msgs::Quaternion im_feat_est_dot_pre_var;
    geometry_msgs::Quaternion im_feat_est_err_var;


    imFeat_estimate << 0,0,1.7,0;
    imFeat_estimate_dot << 0,0,0,0;
    estimation_error << 0, 0, -0.1, 0;
    x1_hat_dot << 0, 0, 0, 0;
    x2_hat_dot << 0, 0, 0, 0;
    x3_hat_dot << 0, 0, 0, 0;
    x4_hat_dot << 0, 0, 0, 0;

    
    gamma_1 = 0.7; //gamma_1 = 0.8; //4th order diff  gamma_1 = 0.7 for 3th order Diff
    gamma_2 = 2*gamma_1 - 1;
    gamma_3 = 3*gamma_1 - 2;
    gamma_4 = 4*gamma_1 - 3;

    delta_1 = 1.2;
    delta_2 = 2*delta_1 - 1;
    delta_3 = 3*delta_1 - 2;
    delta_4 = 4*delta_1 - 3;
    
    /*
    k1 << 6, 6, 6, 6;
    k2 << 6, 6, 6, 6;
    k3 << 8, 8, 8, 8;
    k4 << 10, 10, 10, 10;
    */

    
    k1 << 5, 5, 5, 5;
    k2 << 5, 5, 5, 5;
    k3 << 12, 12, 12, 12;
    

 
    alpha_1 = 0.6;
    alpha_2 = 2*alpha_1 - 1;

    beta_1 = 1.2;
    beta_2 = 2*beta_1 - 1;
 
    G1 << 5, 5, 5, 5;
    G2 << 6, 6, 6, 6;
  
    /*   
    alpha_1 = 0.7;
    alpha_2 = 2*alpha_1 - 1;
    alpha_3 = 3*alpha_1 - 2;

    beta_1 = 1.2;
    beta_2 = 2*beta_1 - 1;
    beta_3 = 3*beta_1 - 2;
 
    G1 << 5, 5, 5, 5;
    G2 << 7, 7, 7, 7;
    G3 << 9, 9, 9, 9;
  */

    im_feat_est_var.x = imFeat_estimate(0);
    im_feat_est_var.y = imFeat_estimate(1);
    im_feat_est_var.z = imFeat_estimate(2);
    im_feat_est_var.w = imFeat_estimate(3);

    im_feat_est_dot_var.x = imFeat_estimate_dot(0);
    im_feat_est_dot_var.y = imFeat_estimate_dot(1);
    im_feat_est_dot_var.z = imFeat_estimate_dot(2);
    im_feat_est_dot_var.w = imFeat_estimate_dot(3);


    im_feat_est_pub.publish(im_feat_est_var);
    im_feat_est_dot_pub.publish(im_feat_est_dot_var);
    ros::Duration(0.1).sleep();

    im_feat_est_var.x = imFeat_estimate(0);
    im_feat_est_var.y = imFeat_estimate(1);
    im_feat_est_var.z = imFeat_estimate(2);
    im_feat_est_var.w = imFeat_estimate(3);

    im_feat_est_dot_var.x = imFeat_estimate_dot(0);
    im_feat_est_dot_var.y = imFeat_estimate_dot(1);
    im_feat_est_dot_var.z = imFeat_estimate_dot(2);
    im_feat_est_dot_var.w = imFeat_estimate_dot(3);

    
    im_feat_est_pub.publish(im_feat_est_var);
    im_feat_est_dot_pub.publish(im_feat_est_dot_var);
    ros::Duration(2.6).sleep();




    while(ros::ok())
    {
        /////// Tracking Differentiator ///////

        //x2_hat_dot = Q1*sig(error,alpha2) + D1*sig(error,beta2)
        //x1_hat_dot = x2_hat + Q2*sig(error,alpha1) + D2*sig(error,beta1)
        //Q1,Q2,D1,D2 > 0 such that the polynomial s^2 + 2s + 2 is Hurwitz
        

        for(int i = 0; i<=3; i++)
        {
            estimation_error(i) = imFeat(i) - imFeat_estimate(i);
/*
            x3_hat_dot(i) = G3(i)*sig(estimation_error(i), alpha_3) + G3(i)*sig(estimation_error(i), beta_3);
            x3_hat(i) = x3_hat(i) + x3_hat_dot(i) * step;

            x2_hat_dot(i) = x3_hat(i) + G2(i)*sig(estimation_error(i), alpha_2) + G2(i)*sig(estimation_error(i), beta_2);
            imFeat_estimate_dot(i) = imFeat_estimate_dot(i) + x2_hat_dot(i)*step;

            x1_hat_dot(i) = imFeat_estimate_dot(i) + G1(i)*sig(estimation_error(i), alpha_1) + G1(i)*sig(estimation_error(i), beta_1);
            imFeat_estimate(i) = imFeat_estimate(i) + x1_hat_dot(i)*step; 
     */
            
            x2_hat_dot(i) = G2(i)*sig(estimation_error(i), alpha_2) + G2(i)*sig(estimation_error(i), beta_2);
            imFeat_estimate_dot(i) = imFeat_estimate_dot(i) + x2_hat_dot(i)*step;

            x1_hat_dot(i) = imFeat_estimate_dot(i) + G1(i)*sig(estimation_error(i), alpha_1) + G1(i)*sig(estimation_error(i), beta_1);
            imFeat_estimate(i) = imFeat_estimate(i) + x1_hat_dot(i)*step;   
         
            /*
            estimation_error2(i) = imFeat_estimate_dot(i) - x1_hat(i);
                       
            x3_hat_dot(i) = k3(i)*sig(estimation_error2(i), gamma_3) + k3(i)*sig(estimation_error2(i), delta_3);
            x3_hat(i) = x3_hat(i) + x3_hat_dot(i) * step;

            new_x2_hat_dot(i) = x3_hat(i) + k2(i)*sig(estimation_error2(i), gamma_2) + k2(i)*sig(estimation_error2(i), delta_2);
            x2_hat(i) = x2_hat(i) + new_x2_hat_dot(i)*step;

            new_x1_hat_dot(i) = imFeat_estimate_dot(i) + k1(i)*sig(estimation_error2(i), gamma_1) + k1(i)*sig(estimation_error2(i), delta_1);
            x1_hat(i) = x1_hat(i) + new_x1_hat_dot(i)*step;  
            */

            /*
            estimation_error2(i) = imFeat_estimate_dot(i) - x1_hat(i);
            
            x4_hat_dot(i) = k4(i)*sig(estimation_error2(i), gamma_4) + k4(i)*sig(estimation_error2(i), delta_4);
            x4_hat(i) = x4_hat(i) + x4_hat_dot(i) * step;
            
            x3_hat_dot(i) = x4_hat(i) + k3(i)*sig(estimation_error2(i), gamma_3) + k3(i)*sig(estimation_error2(i), delta_3);
            x3_hat(i) = x3_hat(i) + x3_hat_dot(i) * step;

            new_x2_hat_dot(i) = x3_hat(i) + k2(i)*sig(estimation_error2(i), gamma_2) + k2(i)*sig(estimation_error2(i), delta_2);
            x2_hat(i) = x2_hat(i) + new_x2_hat_dot(i)*step;

            new_x1_hat_dot(i) = imFeat_estimate_dot(i) + k1(i)*sig(estimation_error2(i), gamma_1) + k1(i)*sig(estimation_error2(i), delta_1);
            x1_hat(i) = x1_hat(i) + new_x1_hat_dot(i)*step;  
            */
            
                  
        }

        im_feat_est_var.x = imFeat_estimate(0);
        im_feat_est_var.y = imFeat_estimate(1);
        im_feat_est_var.z = imFeat_estimate(2);
        im_feat_est_var.w = imFeat_estimate(3);

        im_feat_est_dot_var.x = imFeat_estimate_dot(0);
        im_feat_est_dot_var.y = imFeat_estimate_dot(1);
        im_feat_est_dot_var.z = imFeat_estimate_dot(2);
        im_feat_est_dot_var.w = imFeat_estimate_dot(3);

        im_feat_est_err_var.x = estimation_error(0);
        im_feat_est_err_var.y = estimation_error(1);
        im_feat_est_err_var.z = estimation_error(2);
        im_feat_est_err_var.w = estimation_error(3);

        im_feat_est_dot_pre_var.x = imFeat_estimate_dot(0);
        im_feat_est_dot_pre_var.y = imFeat_estimate_dot(1);
        im_feat_est_dot_pre_var.z = imFeat_estimate_dot(2);
        im_feat_est_dot_pre_var.w = imFeat_estimate_dot(3);

        im_feat_est_pub.publish(im_feat_est_var);
        im_feat_est_dot_pub.publish(im_feat_est_dot_var);
        im_feat_est_err_pub.publish(im_feat_est_err_var);
        im_feat_est_dot_pre_pub.publish(im_feat_est_dot_pre_var);
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}