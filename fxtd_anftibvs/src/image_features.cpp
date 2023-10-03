//Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
//Including opencv libraries
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
//Including Eigen library
#include <eigen3/Eigen/Dense>

//Declaring global variables
cv::Mat frame;

int ug, ug1, ug2, ug3, ug4;
int ng, ng1, ng2, ng3, ng4;
int p1,p2,p3,p4;
int p1x,p2x,p3x,p4x;
int p1y,p2y,p3y,p4y;

Eigen::Vector3f p1_vs_cf;
Eigen::Vector3f p2_vs_cf;
Eigen::Vector3f p3_vs_cf;
Eigen::Vector3f p4_vs_cf;

Eigen::Vector3f p1_vs_vf;
Eigen::Vector3f p2_vs_vf;
Eigen::Vector3f p3_vs_vf;
Eigen::Vector3f p4_vs_vf;

Eigen::Vector3f uav_att;

Eigen::Vector3f e3;

float beta_p1;
float beta_p2;
float beta_p3;
float beta_p4;

float ug_vs, ng_vs;
float mu20,mu02,mu11;
float den;

float qx,qy,qz,qpsi,qyaw;
float a;
//float aD = 0.000001256;
//float aD = 0.0000005556;
float aD = 0.0000007970; //2.5 meters

//Camera intrinsic parameters
float focal_length = 0.00304;
//float pixel_size = 0.00000112; //frame size = raspy cam's default image size
float pixel_size = 0.00000876923; //For frame_size = 410x308
//float new_focal_length = focal_length*pixel_size; 

/////////////////////////Functions///////////////////////////////

//Matrix R_phi_theta
Eigen::Matrix3f Rtp(float roll, float pitch)
{
    Eigen::Matrix3f pitch_mat;
    pitch_mat << cos(pitch), 0.0, sin(pitch),
        0.0, 1.0, 0.0,
        -sin(pitch), 0.0, cos(pitch);


    Eigen::Matrix3f roll_mat;
    roll_mat << 1.0, 0.0, 0.0,
        0.0, cos(roll), -sin(roll),
        0.0, sin(roll), cos(roll);

    Eigen::Matrix3f R_tp;
    R_tp = pitch_mat * roll_mat;

    return R_tp;

}

Eigen::Matrix3f Ryaw(float yaw)
{   
    Eigen::Matrix3f yaw_mat;
    yaw_mat << cos(yaw), -sin(yaw),0,
                sin(yaw), cos(yaw), 0,
                0, 0, 1;
    return yaw_mat;
}

/////////////////////ROS Subscribers//////////////////////////////////
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		frame = cv_bridge::toCvShare(msg, "bgr8")->image; //Saving the image
		//cv::circle(frame, cv::Point(205, 154), 8, cv::Scalar(255, 0, 255));
		//cv::circle(frame, cv::Point(ug1, ng1), 8, cv::Scalar(255, 0, 0));
		//cv::circle(frame, cv::Point(ug2, ng2), 8, cv::Scalar(0, 255, 0));
		//cv::circle(frame, cv::Point(ug3, ng3), 8, cv::Scalar(0, 0, 255));
		//cv::circle(frame, cv::Point(ug4, ng4), 8, cv::Scalar(255, 255, 0));
	}
	
	catch(cv_bridge::Exception& e)
	{ 
		ROS_ERROR("Couldn't convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void attitude_callback(const geometry_msgs::Vector3::ConstPtr& att)
{
	uav_att(0) = att->x;
	uav_att(1) = att->y;
	uav_att(2) = att->z;
}

////////////////////Main program//////////////////////////////////////
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "image_features");
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
	ros::Rate loop_rate(50);	
    
    //ROS publishers and subscribers
	ros::Publisher im_feat_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_vector",100);
    ros::Publisher a_value_pub = nh.advertise<std_msgs::Float64>("a_value",100);
	ros::Publisher u_coord_pub = nh.advertise<geometry_msgs::Quaternion>("u_coordinates",100);
	ros::Publisher n_coord_pub = nh.advertise<geometry_msgs::Quaternion>("n_coordinates",100);
	
    //ros::Publisher p1_pub = nh.advertise<geometry_msgs::Pose2D>("punto1",100);
	//ros::Publisher p2_pub = nh.advertise<geometry_msgs::Pose2D>("punto2",100);
	//ros::Publisher p3_pub = nh.advertise<geometry_msgs::Pose2D>("punto3",100);
	//ros::Publisher p4_pub = nh.advertise<geometry_msgs::Pose2D>("punto4",100);

    image_transport::Subscriber sub = it.subscribe("/quad/camera/image_raw", 100, imageCallback); //Gazebo_camera 
    //image_transport::Subscriber sub = it.subscribe("camera/image", 100, imageCallback); //Real camera
	ros::Subscriber attitude_sub = nh.subscribe("quad_attitude",100, &attitude_callback);
	    
    //Declaring local variables
    geometry_msgs::Quaternion im_feat_vec;
    std_msgs::Float64 a_val;
	geometry_msgs::Quaternion u_cam_coord;
	geometry_msgs::Quaternion n_cam_coord;

	//geometry_msgs::Pose2D punto1_vis;
	//geometry_msgs::Pose2D punto2_vis;
	//geometry_msgs::Pose2D punto3_vis;
	//geometry_msgs::Pose2D punto4_vis;
    e3 << 0,0,1;
    
	//Loading the dictionary where the aruco markers belong to
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);

	qx = 0;
	qy = 0;
	qz = 1;
	qpsi  = 0;
	//Publishing data via Rostopics
    im_feat_vec.x = qx;
    im_feat_vec.y = qy;
	im_feat_vec.z = qz;
    im_feat_vec.w = qpsi;

	im_feat_pub.publish(im_feat_vec);
	ros::Duration(2.1).sleep();

    while (ros::ok())
	{
        //Initializing the detector parameters using default values
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		//Declaring the 2D vectors that contain the aruco's corners and rejected candidates 
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectCandidates;
		//Declaring a vector to save de ID numbers of the detected arucos
		std::vector<int> markerIds;
		
		if(!frame.empty())
		{
			//Detect the markers in the image
		cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectCandidates);
		}
		else
		{
			std::cout << "empty " << std::endl;
			qx = 0;
			qy = 0;
			qz = 1;
			qpsi  = 0;
			 //Publishing data via Rostopics
            im_feat_vec.x = qx;
            im_feat_vec.y = qy;
            im_feat_vec.z = qz;
            im_feat_vec.w = qpsi;

			im_feat_pub.publish(im_feat_vec);
		}

        //If no arucos are detected, then print on screen
		if (markerIds.size() != 4)
		{
            std::cout << "I see " << markerIds.size() << " arucos only. I need 4 to work properly" << std::endl;
			qx = 0;
			qy = 0;
			qz = 1;
			qpsi  = 0;

			 //Publishing data via Rostopics
            im_feat_vec.x = qx;
            im_feat_vec.y = qy;
            im_feat_vec.z = qz;
            im_feat_vec.w = qpsi;

			im_feat_pub.publish(im_feat_vec);
		}

		else if (markerIds.size() == 4)
        {
            //Assignment of Point 1
			if (markerIds[0] == 6)
			{
				//Corner extraction
				cv::Point p11 = markerCorners[0].at(0);
				cv::Point p21 = markerCorners[0].at(1);
				cv::Point p31 = markerCorners[0].at(2);
				cv::Point p41 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			else if (markerIds[1] == 6)
			{
				//Corner extraction
				cv::Point p11 = markerCorners[1].at(0);
				cv::Point p21 = markerCorners[1].at(1);
				cv::Point p31 = markerCorners[1].at(2);
				cv::Point p41 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			else if (markerIds[2] == 6)
			{
				//Corner extraction
				cv::Point p11 = markerCorners[2].at(0);
				cv::Point p21 = markerCorners[2].at(1);
				cv::Point p31 = markerCorners[2].at(2);
				cv::Point p41 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			else if (markerIds[3] == 6)
			{
				//Corner extraction
				cv::Point p11 = markerCorners[3].at(0);
				cv::Point p21 = markerCorners[3].at(1);
				cv::Point p31 = markerCorners[3].at(2);
				cv::Point p41 = markerCorners[3].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			//Assignment of Point 2
			if (markerIds[0] == 4)
			{
				//Corner extraction
				cv::Point p12 = markerCorners[0].at(0);
				cv::Point p22 = markerCorners[0].at(1);
				cv::Point p32 = markerCorners[0].at(2);
				cv::Point p42 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			else if (markerIds[1] == 4)
			{
				//Corner extraction
				cv::Point p12 = markerCorners[1].at(0);
				cv::Point p22 = markerCorners[1].at(1);
				cv::Point p32 = markerCorners[1].at(2);
				cv::Point p42 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			else if (markerIds[2] == 4)
			{
				//Corner extraction
				cv::Point p12 = markerCorners[2].at(0);
				cv::Point p22 = markerCorners[2].at(1);
				cv::Point p32 = markerCorners[2].at(2);
				cv::Point p42 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			else if (markerIds[3] == 4)
			{
				//Corner extraction
                cv::Point p12 = markerCorners[3].at(0);
				cv::Point p22 = markerCorners[3].at(1);
				cv::Point p32 = markerCorners[3].at(2);
				cv::Point p42 = markerCorners[3].at(3);

                //Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			///////////////////////////////////////////////////////////////////////////////77
			//Assignment of Point 3
			if (markerIds[0] == 8)
			{
				//Corner extraction
				cv::Point p13 = markerCorners[0].at(0);
				cv::Point p23 = markerCorners[0].at(1);
				cv::Point p33 = markerCorners[0].at(2);
				cv::Point p43 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			else if (markerIds[1] == 8)
			{
				//Corner extraction
				cv::Point p13 = markerCorners[1].at(0);
				cv::Point p23 = markerCorners[1].at(1);
				cv::Point p33 = markerCorners[1].at(2);
				cv::Point p43 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			else if (markerIds[2] == 8)
			{
				//Corner extraction
				cv::Point p13 = markerCorners[2].at(0);
				cv::Point p23 = markerCorners[2].at(1);
				cv::Point p33 = markerCorners[2].at(2);
				cv::Point p43 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			else if (markerIds[3]==8)
			{
				//Corner extraction
				cv::Point p13 = markerCorners[3].at(0);
				cv::Point p23 = markerCorners[3].at(1);
				cv::Point p33 = markerCorners[3].at(2);
				cv::Point p43 = markerCorners[3].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
		////////////////////////////////////////////////////////////////////////////////////7
			//Assignment of Point 4
			if (markerIds[0] == 10)
			{
				//Corner extraction
        		cv::Point p14 = markerCorners[0].at(0);
				cv::Point p24 = markerCorners[0].at(1);
				cv::Point p34 = markerCorners[0].at(2);
				cv::Point p44 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			else if (markerIds[1] == 10)
			{
				//Corner extraction
				cv::Point p14 = markerCorners[1].at(0);
				cv::Point p24 = markerCorners[1].at(1);
				cv::Point p34 = markerCorners[1].at(2);
				cv::Point p44 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			else if (markerIds[2] == 10)
			{
				//Corner extraction
				cv::Point p14 = markerCorners[2].at(0);
				cv::Point p24 = markerCorners[2].at(1);
				cv::Point p34 = markerCorners[2].at(2);
				cv::Point p44 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			else if (markerIds[3] == 10)
			{
				//Corner extraction
				cv::Point p14 = markerCorners[3].at(0);
				cv::Point p24 = markerCorners[3].at(1);
				cv::Point p34 = markerCorners[3].at(2);
				cv::Point p44 = markerCorners[3].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			
            //Centroid of the target
			ug = (ug1 + ug2 + ug3 + ug4) / 4;
			ng = (ng1 + ng2 + ng3 + ng4) / 4;
			
            /* Indicators
			cv::circle(frame, cv::Point(ug, ng), 8, cv::Scalar(255, 0, 255));
			cv::circle(frame, cv::Point(ug1, ng1), 8, cv::Scalar(255, 0, 0));
			cv::circle(frame, cv::Point(ug2, ng2), 8, cv::Scalar(0, 255, 0));
			cv::circle(frame, cv::Point(ug3, ng3), 8, cv::Scalar(0, 0, 255));
			cv::circle(frame, cv::Point(ug4, ng4), 8, cv::Scalar(255, 255, 0));
			*/
			

            //Declaring the points required for visual servoing
			cv::Point p1 = cv::Point(ug1,ng1);
			cv::Point p2 = cv::Point(ug2,ng2);
			cv::Point p3 = cv::Point(ug3,ng3);
			cv::Point p4 = cv::Point(ug4,ng4);

            //Changing Image coordinates system from the left-top to the center.
			/*
											+y	^		
												|
												|
												|
			      					-x <--------|---------> +x
					      						|
			      								|
			      								|		
			      								|
			      								-y		
					
			*/
			p1x = p1.x-frame.size().width/2;
			p1y = -(p1.y-frame.size().height/2);
			
			p2x = p2.x-frame.size().width/2;
			p2y = -(p2.y-frame.size().height/2);
			
			p3x = p3.x-frame.size().width/2;
			p3y = -(p3.y-frame.size().height/2);
			
			p4x = p4.x-frame.size().width/2;
			p4y = -(p4.y-frame.size().height/2);

            //Camera frame point data (u,n,focal_length)
            p1_vs_cf << p1x*pixel_size, p1y*pixel_size, focal_length;
			p2_vs_cf << p2x*pixel_size, p2y*pixel_size, focal_length;
			p3_vs_cf << p3x*pixel_size, p3y*pixel_size, focal_length;
			p4_vs_cf << p4x*pixel_size, p4y*pixel_size, focal_length;


            //Camera frame to Virtual frame conversion
			beta_p1 = focal_length/(e3.transpose()*Rtp(uav_att(0),uav_att(1))*p1_vs_cf);
			beta_p2 = focal_length/(e3.transpose()*Rtp(uav_att(0),uav_att(1))*p2_vs_cf);
			beta_p3 = focal_length/(e3.transpose()*Rtp(uav_att(0),uav_att(1))*p3_vs_cf);
			beta_p4 = focal_length/(e3.transpose()*Rtp(uav_att(0),uav_att(1))*p4_vs_cf);

            p1_vs_vf = beta_p1*Rtp(uav_att(0),uav_att(1))*p1_vs_cf;
			p2_vs_vf = beta_p2*Rtp(uav_att(0),uav_att(1))*p2_vs_cf;
			p3_vs_vf = beta_p3*Rtp(uav_att(0),uav_att(1))*p3_vs_cf;
			p4_vs_vf = beta_p4*Rtp(uav_att(0),uav_att(1))*p4_vs_cf;

			/*
			p1_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p1_vs_vf;
			p2_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p2_vs_vf;
			p3_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p3_vs_vf;
			p4_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p4_vs_vf;
			*/	
            //Ordinary moments. Centroid
			ug_vs = (p1_vs_vf(0) + p2_vs_vf(0) + p3_vs_vf(0) + p4_vs_vf(0)) / 4;
			ng_vs = (p1_vs_vf(1) + p2_vs_vf(1) + p3_vs_vf(1) + p4_vs_vf(1)) / 4;
			
			//Momentos centrados
			mu20 = powf((p1_vs_vf(0) - ug_vs), 2) +  powf((p2_vs_vf(0) - ug_vs), 2) +  powf((p3_vs_vf(0) - ug_vs), 2) +  powf((p4_vs_vf(0) - ug_vs), 2);
			mu02 = powf((p1_vs_vf(1) - ng_vs), 2) +  powf((p2_vs_vf(1) - ng_vs), 2) +  powf((p3_vs_vf(1) - ng_vs), 2) +  powf((p4_vs_vf(1) - ng_vs), 2);
			mu11 = ((p1_vs_vf(0) - ug_vs) * (p1_vs_vf(1) - ng_vs)) + ((p2_vs_vf(0) - ug_vs) * (p2_vs_vf(1) - ng_vs)) + ((p3_vs_vf(0) - ug_vs) * (p3_vs_vf(1) - ng_vs)) + ((p4_vs_vf(0) - ug_vs) * (p4_vs_vf(1) - ng_vs));
			den = mu20-mu02;

            //Image features vector
			a = mu20 + mu02;
			qz = sqrt(aD/a);
			qx = qz * ng_vs/focal_length;
			qy = qz * ug_vs/focal_length;
			qpsi = -0.5 * atan(2*mu11/den);

            //Publishing data via Rostopics
            im_feat_vec.x = qx;
            im_feat_vec.y = qy;
            im_feat_vec.z = qz;
            im_feat_vec.w = qpsi;

			u_cam_coord.x = p1_vs_vf(0)/pixel_size;
			u_cam_coord.y = p2_vs_vf(0)/pixel_size;
			u_cam_coord.z = p3_vs_vf(0)/pixel_size;
			u_cam_coord.w = p4_vs_vf(0)/pixel_size;

			n_cam_coord.x = p1_vs_vf(1)/pixel_size;
			n_cam_coord.y = p2_vs_vf(1)/pixel_size;
			n_cam_coord.z = p3_vs_vf(1)/pixel_size;
			n_cam_coord.w = p4_vs_vf(1)/pixel_size;

            a_val.data = a;			

           	im_feat_pub.publish(im_feat_vec);
            a_value_pub.publish(a_val);
			u_coord_pub.publish(u_cam_coord);
			n_coord_pub.publish(n_cam_coord);
			
			/*
			std::cout << "p1 " << p1_vs_vf(0) << ", " << p1_vs_vf(1) << std::endl;
			std::cout << "p2 " << p2_vs_vf(0) << ", " << p2_vs_vf(1) << std::endl;
			std::cout << "p3 " << p3_vs_vf(0) << ", " << p3_vs_vf(1) << std::endl;
			std::cout << "p4 " << p4_vs_vf(0) << ", " << p4_vs_vf(1) << std::endl;
			std::cout << "mu11 " << mu11 << std::endl;*/
			std::cout << "OK" << std::endl;
			
        }

		ros::spinOnce();
		loop_rate.sleep();
	}  

    return 0;
}