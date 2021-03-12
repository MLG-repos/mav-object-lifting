#ifndef CONTROL_FRONTAL
#define CONTROL_FRONTAL

// Introduccion a la Robotica
// Manuel Lopez Garcia
// 2019.02.11
// Lista de cambios:
// 1. Agregadas variables para asignar ganancias kp, ki, kd para Yaw, Altitude, Distance
// 2. Agregadas variables de soporte para errores PID para Yaw, Altitude, Distance

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Gazebo
#include <sensor_msgs/Range.h>  //sonar
#include <sensor_msgs/Imu.h>  //odometry
#include <gazebo_msgs/ModelStates.h>  //Pose from gazebo
#include <gazebo_msgs/SetModelState.h>

// Bebop 2
#include <nav_msgs/Odometry.h>
#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>

// Synchronize events
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#include <vector>

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <time.h>

using namespace cv;
using namespace std;

//using namespace message_filters;

enum MovStates {
    MOV_YAW_ALT = 0,
    MOV_DIST,
    MOV_STANDBY
};

enum MovStatesPID2 {
    MOV2_YAW = 0,
    MOV2_ALT,
    MOV2_SIDE,
    MOV2_FRONT,
    MOV2_STANDBY
};

enum MovStatesPID4 {
	MOV4_SELECT = 0,
    MOV4_ALT,
    MOV4_YAW_OBJECT,
    MOV4_SIDE,
    MOV4_FRONT,
    MOV4_STANDBY
};

struct Navigation_ {
    cv::Point TrunkCenter;
    cv::Point TrunkCenter1;
    cv::Point TrunkCenter2;
    int TrunkHeight;
    int TrunkHeight1;
    int TrunkHeight2;

    ulong ContoursCount; // trunks count

    int RoadCenterListX[10];
    int RoadCenterListY[10];
    cv::Point RoadCenterSmooth;
    double RoadAngle;

    cv::Point CameraCenter;
    int CameraFramesCount;

    int StartX;
    int StartY;
    int StartFromRight;
    int StepCount;
    int StepMax;
    double Step;
    double Angle; // por defecto

    int State;
};

struct Inspection_ {
    int SideTrunkWidthRef;
    cv::Rect SideTrunk;
    double PID_K;
};

struct Map_ {
    int SlideTreeCount;
    int CorridorCount;

    cv::Point Pt1;
    cv::Point Pt2;
    cv::Mat Mini;
};

struct PID_ {
    std::vector<double> References;
    double Current;

    struct {
        double Error;
        double MeanError;
        double LastError;
        double Integral;
        double IntegralWindupGuard;
        double Derivative;
        double DeltaT;
        double Output;
        bool FirstDerivative;

        double Kp;
        double Ki;
        double Kd;

        tf::Matrix3x3 Rt;
        tf::Matrix3x3 Rd;
    } Pid;
};

struct Trajectory_ {
    double DeltaX;
    double DeltaY;
    double DeltaZ;
    double ShiftZ; // desplazamiento de XY por cada cambio en Z
    int XMax;
    int YMax;
    int ZMax;
    int XCnt;
    int YCnt;
    int ZCnt;
    double XIni;
    double YIni;
    double ZIni;
    int State;
    int Times;
    int TimesCnt;
    int DiscardFrames;

    int RangeRoll;
    int RangePitch;
    int RangeYaw;

    double Roll;
    double Pitch;
    double Yaw;
    double DeltaRoll;
    double DeltaPitch;
    double DeltaYaw;
};

struct RVizBasicShapes_ {
    //ros::Publisher MarkerPub;
    uint32_t Shape;
    visualization_msgs::Marker Mark;
    int TimesCnt;
    float F;
};

struct RVizPointsLines_ {
    //ros::Publisher MarkerPub;
    visualization_msgs::Marker Points;
    visualization_msgs::Marker LineStrip;
    visualization_msgs::Marker LineList;
    int TimesCnt;
    double F;
};

struct RVizDrone_ {
    //ros::Publisher MarkerPub;
    visualization_msgs::Marker LineStrip;
    int TimesCnt;
};

struct Kalman_ {
    double X00;
    double X10;
    double P00;
    double P10;
    double Q;
    double R1;
    double K;
    double X11;
    double P11;
};

struct ObjLocMsg_ {
    double Xc;
    double Yc;
    double H;
    double W;
    tf::Quaternion Q;
    double SF;
    double Score;
    
    double ImgW;
    double ImgH;
    int XcPixel;
    int YcPixel;

    double Roll;
    double Pitch;
    double Yaw;
    double Altitude; // meter

    // Mean filter buffers
    double XcFiltered;
    double YcFiltered;
    double YawFiltered;
    double SFFiltered;
    double QwFiltered;
    double QxFiltered;
    double QyFiltered;
    double QzFiltered;

    // Alternative Kalman filters
    // Kalman filter
    struct Kalman_ YawKalman;
    struct Kalman_ SFKalman;
    struct Kalman_ XcKalman;
    struct Kalman_ YcKalman;

    // thresholds for PID control
    double SideThrPixel; // pixels
    double FrontThrPixel; // pixels
    double SideThr; // normalized
    double FrontThr; // normalized
    double AltThr; // meters
    double YawThr; // degrees
    double ScoreThr;
};

struct Metrics_
{
    int AccuracyTotal;
    int AccuracyFail;

    clock_t TimeIni, TimeEnd;
};

class ControlFrontal {
//private:
public:
    ros::NodeHandle nh;

    image_transport::ImageTransport MavImgTransport;
    image_transport::Subscriber MavImgSubs;
    Mat MavImg;
    cv_bridge::CvImagePtr MavImgCvBridge;

    ros::Publisher MavCommandPub;
    geometry_msgs::Twist MavCommand; //velocities

    // Gazebo pose
    ros::Subscriber GazeboPoseSubs;
    geometry_msgs::Pose GazeboPose;

    // PoseNet pose
    ros::Subscriber PoseNetSubs;
    geometry_msgs::Pose PoseNetPose;
    int PoseNetFilterSize;

    // Object Localization Neural Network
    ros::Subscriber ObjLocSubs;
    struct ObjLocMsg_ ObjLocMsg;
    
    ros::Subscriber ObjLocGrasperSubs;
    struct ObjLocMsg_ ObjGrasperMsg; // only uses coordinates

    struct Metrics_ Metrics;

    // VICON
    ros::Subscriber ViconBebopSubs;
    geometry_msgs::TransformStamped ViconBebopPose;
    ros::Subscriber ViconBoteSubs;
    geometry_msgs::TransformStamped ViconBotePose;

    // SYNC
    //message_filters::TimeSynchronizer<image_transport::ImageTransport, ros::Subscriber> TimeSync;
    //image_transport::SubscriberFilter ImageSubscriber;
    //typedef message_filters::sync_policies::ApproximateTime<image_transport::SubscriberFilter, ros::Subscriber> MySyncPolicy;
    //typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    //boost::shared_ptr<Sync> Sync_;

    // Gazebo SetPose
    ros::NodeHandle ModelToSetNode;
    geometry_msgs::Pose ModelToSetPose;
    geometry_msgs::Twist ModelToSetTwist;
    gazebo_msgs::ModelState ModelToSet;
    ros::ServiceClient ModelToSetClient;
    gazebo_msgs::SetModelState ModelToSetState;

    struct Trajectory_ Traj;
    struct RVizBasicShapes_ RVizBasicShapes;
    struct RVizPointsLines_ RVizPointsLines;
    struct RVizDrone_ RVizDrone;

    // ardrone imu / Bebop 2 odometry
    ros::Subscriber ARdroneOdomSubs;
    //sensor_msgs::Imu odom;
    double ARdroneOdomQx;
    double ARdroneOdomQy;
    double ARdroneOdomQz;
    double ARdroneOdomQw;

    double Roll; // no need for PID
    double Pitch; // no need for PID
    struct PID_ Yaw;
    struct PID_ Side;
    struct PID_ Front;
    struct PID_ Altitude;

    // ar drone altitude
    ros::Subscriber AltitudeSubs;
    //sensor_msgs::Range range;

    double dist_x; // TODO local var
    double dist_y; // TODO local var
    double PoseXi;
    double PoseYi;
    double PoseX;
    double PoseY;
    double PoseZ;

    // maquina de estados global
    ros::Subscriber OverrideSubs;
    int AutoControl;
    int MovState;
    uint32_t MovIndex;
    int GazeboModelID;
    string MsgStr;

    // auxiliares para salvar dataset Bote
    tf::Quaternion Q;
    std::vector<std::string> DataLine;
    int ImageCount;
    int SavePending;
    bool SaveOn;
    int SaveEnable;

    struct Map_ Map;
    struct Navigation_ Nav;
    struct Inspection_ Insp;

    // Control By Bucket-Shaped Load
    double RollIntegral;
    double PitchIntegral;
    bool UseBias;
    bool ControlSEnable;
    bool YawReached;
    
    int HoveringOffCounter;
    int HoveringCounter;
    int HoveringSteps;
    bool HoveringRoll;
    bool HoveringPitch;
    bool HoveringEnable;
    bool HoveringSignal;

    // Control By Intersection of bucket and grasper
    bool ControlBIEnable;
    bool ControlBIPidEnable;
    bool ControlBIBangEnable;
    int ControlBIBangCounter;
    int ControlBIBangState;
    double ControlBIBangSideOut;
    double ControlBIBangFrontOut;
    int ControlBIMaxCounter;
    int ControlBICounter;
    double ControlBIAcc;
    double ControlBISpeed;
    
    // Lift Control, used by the two previous control methods
    bool LiftEnable;
    int LiftCounter;
    int LiftPositiveCounter;
    int LiftSteps;
    int LiftState;
    	
public:
    ControlFrontal();
    ~ControlFrontal();

    void Image_Callback(const sensor_msgs::ImageConstPtr &msg);
    void Image_Callback2(const sensor_msgs::ImageConstPtr &msg);
    void Image_Callback3(const sensor_msgs::ImageConstPtr &msg);
    void Image_Callback4(const sensor_msgs::ImageConstPtr &msg);
    void Gazebo_Pose_Callback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    //~ void Ardrone_Altitude_Callback(const sensor_msgs::Range::ConstPtr &range);
    void Ardrone_Altitude_Callback(const bebop_msgs::Ardrone3PilotingStateAltitudeChanged &alt);

    void Ardrone_Odom_Callback(const sensor_msgs::Imu::ConstPtr &odom);
    void Bebop2_Odom_Callback(const nav_msgs::Odometry::ConstPtr &odom);
    void Keyboard_Callback(const std_msgs::Int8 &flag);
    void Vicon_BebopCallback(const geometry_msgs::TransformStamped::ConstPtr &pose);
    void Vicon_BoteCallback(const geometry_msgs::TransformStamped::ConstPtr &pose);
    
    void ControlS(void);
    void ControlLift(void);
    void ControlLift2(void);
    void ControlByIntersection(void);
    void ControlByIntersection2(void);
    void ControlByIntersection3(void);
    void ControlHoveringSignal(void);
    
    void PID_Control5(void);
    void PID_Control6(void);
    void PID_Control6A(void);
    void PID_Control6B(void);
    
    void PoseNet_Callback(const geometry_msgs::Pose::ConstPtr &pose);
    double PoseNet_AverageFilter(double newVal, std::vector<double> &buffer, uint32_t filterSize);
    
    void ObjLoc_BucketCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void ObjLoc_DrawBucketWithYaw(cv::Mat &img);
    void ObjLoc_DrawBucket(cv::Mat &img);
    void ObjLoc_RotatePoint(cv::Point &p_rot, cv::Point pc, cv::Point p, double a);
    void ObjLoc_GrasperCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void ObjLoc_DrawGrasper(cv::Mat &img);

    int Utils_RandomAngle(int range);
    int Utils_RandomVal(int low, int high);
    void Utils_Q2Y(tf::Quaternion &Q, double &yaw);
    void Utils_KalmanInit(struct Kalman_ &kalman, double x00, double p00, double q, double r1);
    void Utils_KalmanUpdate(struct Kalman_ &kalman, double z1);
    double Utils_MeanFilter(double newVal, double &mean_1, double filterSize);
    void Utils_ImageCenterCrop(cv::Mat &img, int outW, int outH);
    double Utils_AngleBetweenLines(int x1, int y1, int x2, int y2);
};

#endif
