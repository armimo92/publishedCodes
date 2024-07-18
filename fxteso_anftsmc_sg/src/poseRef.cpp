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

float x_des = 0;
float y_des = 0;
float z_des = 0;
float yaw_des = 0;
float x_dot_des = 0;
float y_dot_des = 0;
float z_dot_des = 0;
float z_ddot_des = 0;
float yawRate = 0;
float step = 0.01;
int i = 0;
float t = 0;

Eigen::Vector3f position(0,0,0);

void posCallback(const geometry_msgs::Vector3::ConstPtr& p)
{
    position(0) = p->x;
    position(1) = p->y;
    position(2) = p->z;
}

using namespace std;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "poseRef");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	
   
    ros::Publisher posRef_pub = nh.advertise<geometry_msgs::Vector3>("desired_position",100);
    ros::Publisher velRef_pub = nh.advertise<geometry_msgs::Quaternion>("desired_velocity",100);
    ros::Publisher yawRef_pub = nh.advertise<std_msgs::Float64>("desired_yaw",100);
    ros::Publisher att_vel_des_pub = nh.advertise<geometry_msgs::Vector3>("desired_attitude_velocity",100);

    ros::Subscriber pos_sub = nh.subscribe("quad_position", 100, &posCallback);

    geometry_msgs::Vector3 posRef_var;
    geometry_msgs::Quaternion velRef_var;
    std_msgs::Float64 yawRef_var;
    geometry_msgs::Vector3 attVelRef_var;

    posRef_var.x = 0;
    posRef_var.y = 0;
    posRef_var.z = 0;

    velRef_var.x = 0;
    velRef_var.y = 0;
    velRef_var.z = 0;
    velRef_var.w = 0;

    yawRef_var.data = 0;

    attVelRef_var.x = 0;
    attVelRef_var.y = 0;
    attVelRef_var.z = 0;

    posRef_pub.publish(posRef_var);
    velRef_pub.publish(velRef_var);
    yawRef_pub.publish(yawRef_var);
    att_vel_des_pub.publish(attVelRef_var);

    // ifstream myFile;
    // string line;
    // myFile.open("/home/armando/Documents/uav_ws/src/fxteso_ansftsmc/src/csv_files/traj.csv");
    // getline(myFile, line);

    ros::Duration(2).sleep();
  /*  while (getline(myFile, line) && ros::ok())
    {

        t = i*step;

        if(position(2) == 0)
        {
            x_des = 0;
            y_des = 0;
            yaw_des = 0;
            x_dot_des = 0;
            y_dot_des = 0;
            z_dot_des = 0;
            yawRate = 0;
            i = 0;
        }

        
        stringstream stream(line); // Convertir la cadena a un stream
        string x, x_dot, y, y_dot;
        // Extraer todos los valores de esa fila
        getline(stream, x, ',');
        getline(stream, x_dot, ',');
        getline(stream, y, ',');
        getline(stream, y_dot, ',');

        x_des = stof(x);
        x_dot_des = stof(x_dot);
        y_des = stof(y);
        y_dot_des = stof(y_dot);

        if(t < 15)
        {
            z_dot_des = 0;
        }
        else if(t >= 15 && t< 60)
        {
            z_dot_des = -0.2;
        }         
        else if (t>=60 && t<90)
        {
            z_dot_des = 0;
        }
        else if (t >= 90 && t < 135)
        {
            z_dot_des = 0.2;
        }
        else if (t >= 135)
        {
            z_dot_des = 0;
        }

        if(t < 15)
        {
            z_des = -2;
        }
        else if (t>=15)
        {
            z_des = z_des + z_dot_des*step;         
        }
        
        if (z_des >0)
        {
            z_des  = 0;
        }

        z_ddot_des = 0;

        yaw_des = 0;
        yawRate = 0;

        posRef_var.x = x_des;
        posRef_var.y = y_des;
        posRef_var.z = z_des;

        velRef_var.x = x_dot_des;
        velRef_var.y = y_dot_des;
        velRef_var.z = z_dot_des;
        velRef_var.w = z_ddot_des;

        yawRef_var.data = yaw_des;

        attVelRef_var.x = 0;
        attVelRef_var.y = 0;
        attVelRef_var.z = yawRate;

        posRef_pub.publish(posRef_var);
        velRef_pub.publish(velRef_var);
        yawRef_pub.publish(yawRef_var);
        att_vel_des_pub.publish(attVelRef_var);

        i = i+1;

        ros::spinOnce();
		loop_rate.sleep();        
    }
    myFile.close();
*/
    while(ros::ok())
    {             
        
        t = i*step;
        
        if(t<5)
        {
            x_dot_des = 0;
            y_dot_des = 0;
            yawRate = 0;
        }
        else
        {
            x_dot_des = cos((0.1)*(t-5));
            y_dot_des = sin((0.1)*(t-5));
            yawRate = 0.1;
        }

        if(t < 15)
        {
            z_dot_des = 0;
        }
        else if(t >= 15 && t< 60)
        {
            z_dot_des = -0.2;
        }         
        else if (t>=60 && t<90)
        {
            z_dot_des = 0;
        }
        else if (t >= 90 && t < 135)
        {
            z_dot_des = 0.2;
        }
        else if (t >= 135)
        {
            z_dot_des = 0;
        }


        x_des = x_des + x_dot_des*step;
        y_des = y_des + y_dot_des*step;
        yaw_des = yaw_des + yawRate*step;
        if(t < 15)
        {
            z_des = -2;
        }
        else if (t>=15)
        {
            z_des = z_des + z_dot_des*step;         
        }
        
        if (z_des >0)
        {
            z_des  = 0;
        }
/*    
        x_des = 0;
        y_des = 0;
        z_des = -2;

        x_dot_des = 0;
        y_dot_des = 0;
        z_dot_des = 0;

        yaw_des = 0;
        yawRate = 0;
*/
        posRef_var.x = x_des;
        posRef_var.y = y_des;
        posRef_var.z = z_des;

        velRef_var.x = x_dot_des;
        velRef_var.y = y_dot_des;
        velRef_var.z = z_dot_des;
        velRef_var.w = 0;

        yawRef_var.data = yaw_des;

        attVelRef_var.x = 0;
        attVelRef_var.y = 0;
        attVelRef_var.z = yawRate;

        posRef_pub.publish(posRef_var);
        velRef_pub.publish(velRef_var);
        yawRef_pub.publish(yawRef_var);
        att_vel_des_pub.publish(attVelRef_var);

        i = i+1;

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
