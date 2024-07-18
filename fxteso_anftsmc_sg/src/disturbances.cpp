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

float dist_x = 0;
float dist_y = 0;
float dist_z = 0;
float t = 0;
float step = 0.01;
int i = 0;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "disturbances");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
    
    //ROS publishers and subscribers
    ros::Publisher disturbances_pub = nh.advertise<geometry_msgs::Vector3>("disturbances",100);
    ros::Publisher comp_dist_pub = nh.advertise<geometry_msgs::Vector3>("comp_dist",100);
    geometry_msgs::Vector3 disturbances_var;
    geometry_msgs::Vector3 comp_dist_var;

    ifstream myFile;
    string line;
    myFile.open("/home/armando/Documents/uav_ws/src/fxteso_ansftsmc/src/csv_files/pert_new.csv");
    getline(myFile, line);

    ros::Duration(2).sleep();

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

    /*
    ifstream myFile;
    string line;
    myFile.open("/home/armando/Documents/visual_servoing_ws/src/ibvs_nftasmc/src/wind_moderate_vector.csv");
    getline(myFile, line);
    
    while(ros::ok())
    {
        dist_x = 0;
        dist_y = 0;

        t = i * step;

        if(t >= 10 && t<13)
        {
            dist_z = 1;
        }
        else
        {
            dist_z = 0;
        }
            
        i = i+1;

        disturbances_var.x = dist_x;
        disturbances_var.y = dist_y;
        disturbances_var.z = dist_z;
        disturbances_pub.publish(disturbances_var);

        comp_dist_var.x = dist_x;
        comp_dist_var.y = dist_y;
        comp_dist_var.z = dist_z;
        comp_dist_pub.publish(comp_dist_var);

        ros::spinOnce();
		loop_rate.sleep();        
    }
    //myFile.close();
    */

    return 0;
}

