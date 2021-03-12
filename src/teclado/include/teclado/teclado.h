#ifndef TECLADO_H 
#define TECLADO_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/Int8.h>
#include "std_msgs/Empty.h"

#include <QWidget>
#include <QtGui>

#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QFont>
#include <QKeyEvent>

using namespace std;

class KeyPress: public QWidget 
{

  public:
    KeyPress(QWidget *parent = 0);//constructor
    ~KeyPress();//destructor
	
  protected:
    void keyPressEvent(QKeyEvent * e);
  
  private:
  
  	ros::NodeHandle nh_;
  	
    // MAV control
	ros::Publisher pubLand1_;
	ros::Publisher pubTakeoff1_;
	ros::Publisher pubCommandPilot1_;
	geometry_msgs::Twist commandPilot;
	std_msgs::Empty msgLand;
	std_msgs::Empty msgTakeoff;
    float speed;
    float altitude_speed;
    float pitch; // x?
    float roll; // y?
    float yaw;
    float altitude; // z?

    // Auto Control On-Off mode
    ros::Publisher OverridePub;
	std_msgs::Int8 override;
	
    // MAV's camera control
    ros::Publisher CameraPub;
    geometry_msgs::Twist CameraTwist;
    double CameraAngY; // pan, tilt?
    //double CameraAngX; // tilt
	
    // User Interface
	QGridLayout *grid;
	QLabel *Speed;
	QLabel *Alt_speed;
	QLabel *Fovea_y;
	
	QLabel *Speed_value;
	QLabel *Alt_speed_value;
	QLabel *Fovea_value_y;
	QLabel *Command;

	QFont font;
};

#endif
