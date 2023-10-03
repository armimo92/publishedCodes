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
#include <fstream>
#include <string>
#include <sstream>
//Including Eigen library
#include <eigen3/Eigen/Dense>

using namespace std;

float dist_x;
float dist_y;
float dist_z;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "disturbances");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
    
    //ROS publishers and subscribers
    ros::Publisher disturbances_pub = nh.advertise<geometry_msgs::Vector3>("disturbances",100);
    geometry_msgs::Vector3 disturbances_var;

    ifstream myFile;
    string line;
    myFile.open("/home/armando/Documents/visual_servoing_ws/src/td_ibvs_nsftasmc/src/csv_files/dist1.csv");
    //myFile.open("/home/armando/Documents/visual_servoing_ws/src/td_ibvs_nsftasmc/src/csv_files/new_dist20.csv");
    getline(myFile, line);
    	
    disturbances_var.x = 0;
    disturbances_var.y = 0;
    disturbances_var.z = 0;
    disturbances_pub.publish(disturbances_var);
    ros::Duration(3).sleep();
    disturbances_var.x = 0;
    disturbances_var.y = 0;
    disturbances_var.z = 0;
    disturbances_pub.publish(disturbances_var);
    while (getline(myFile, line) && ros::ok())
    {       
        stringstream stream(line); // Convertir la cadena a un stream
        string x, y, z;
        // Extraer todos los valores de esa fila
        getline(stream, x, ',');
        getline(stream, y, ',');
        getline(stream, z, ',');

        dist_x = stof(x);
        dist_y = stof(y);
        dist_z = stof(z);

        disturbances_var.x = dist_x;
        disturbances_var.y = dist_y;
        disturbances_var.z = dist_z;

        disturbances_pub.publish(disturbances_var);

        ros::spinOnce();
		loop_rate.sleep();        
    }
    
    myFile.close();
    
    while (ros::ok())
    {
        disturbances_var.x = 0;
        disturbances_var.y = 0;
        disturbances_var.z = 0;
        disturbances_pub.publish(disturbances_var);
        ros::spinOnce();
		loop_rate.sleep(); 
    }

   
    return 0;
}

