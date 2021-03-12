/*********************************************************************** 
 * Derechos reservados                                                 *
 * Autores: Leticia Oyuki Rojas Perez, Jose Martinez Carranza          * 
 * Versión: 1.0                                                        *
 * Última actualización:  05/06/2018                                   *
 * Curso EIR 2019                                                      *  
 *                                                                     *
 * Ejemplo de control de frontal del vehículo Ar drone simulado        *
 * usando GAZEBO      
 * 
 * Introduccion a la Robotica
 * Manuel Lopez Garcia
 * 2019.02.11
 * Lista de cambios:
 * 1. Variables de soporte para errores PID inicializadas
 * 2. Conexion de constantes de ganancias Kp, Ki, Kd al archivo config.yaml
 * 3. Agregadas las ecuaciones de control integral para
 *    Yaw, Altitude, Distance
 ***********************************************************************/
 
#include "control_frontal.h"

#define BEBOP2


ControlFrontal::ControlFrontal(): nh("~"), MavImgTransport(nh)
{
    ROS_INFO("Init ControlFrontal");

#ifdef BEBOP2
    MavImgSubs = MavImgTransport.subscribe("/bebop/image_raw", 1, &ControlFrontal::Image_Callback4, this); // bebop 2
    ARdroneOdomSubs = nh.subscribe("/bebop/odom", 10, &ControlFrontal::Bebop2_Odom_Callback, this); // Bebop 2
    AltitudeSubs = nh.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged", 10, &ControlFrontal::Ardrone_Altitude_Callback, this);
    MavCommandPub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 3); // bebop 2
#else
    MavImgSubs = MavImgTransport.subscribe("/camera/image_raw", 1, &ControlFrontal::Image_Callback, this); // tum_simulator
    GazeboPoseSubs = nh.subscribe("/gazebo/model_states", 1, &ControlFrontal::Gazebo_Pose_Callback, this); //1 antes 10, tum_simulator
    ARdroneOdomSubs = nh.subscribe("/ardrone/imu", 10, &ControlFrontal::Ardrone_Odom_Callback, this); // tum_simulator
    MavCommandPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 3); // tum_simulator
#endif

    //PoseNetSubs = nh.subscribe("/viz_posenet", 1, &ControlFrontal::PoseNet_Callback, this);
    ObjLocSubs = nh.subscribe("/obj_loc", 1, &ControlFrontal::ObjLoc_BucketCallback, this);
    ObjLocGrasperSubs = nh.subscribe("/obj_loc_grasper", 1, &ControlFrontal::ObjLoc_GrasperCallback, this);
    OverrideSubs = nh.subscribe("/keyboard/override", 1, &ControlFrontal::Keyboard_Callback, this);

    ViconBebopSubs = nh.subscribe("/vicon/bebop_manuel/bebop_manuel", 1, &ControlFrontal::Vicon_BebopCallback, this);
    //ViconBoteSubs = nh.subscribe("/vicon/bebop_manuel/bote1_manuel", 1, &ControlFrontal::Vicon_BoteCallback, this);

    // drone velocities
    MavCommand.angular.x = 0;
    MavCommand.angular.y = 0;
    MavCommand.angular.z = 0;
    MavCommand.linear.x = 0;
    MavCommand.linear.y = 0;
    MavCommand.linear.z = 0;

    //------------- REFERENCES & PIDs ---------------------------

    Front.References.clear();
    Side.References.clear();
    Yaw.References.clear();
    Altitude.References.clear();
	
    nh.getParam("YawReferences", Yaw.References);
    nh.getParam("YawKp", Yaw.Pid.Kp);
    nh.getParam("YawKi", Yaw.Pid.Ki);
    nh.getParam("YawKd", Yaw.Pid.Kd);
    nh.getParam("YawIntegralWindupGuard", Yaw.Pid.IntegralWindupGuard);
    nh.getParam("PidDeltaT", Yaw.Pid.DeltaT);

    nh.getParam("FrontReferences", Front.References);
    nh.getParam("FrontKp", Front.Pid.Kp);
    nh.getParam("FrontKi", Front.Pid.Ki);
    nh.getParam("FrontKd", Front.Pid.Kd);
    nh.getParam("FrontIntegralWindupGuard", Front.Pid.IntegralWindupGuard);
    nh.getParam("PidDeltaT", Front.Pid.DeltaT);

    nh.getParam("SideReferences", Side.References);
    nh.getParam("SideKp", Side.Pid.Kp);
    nh.getParam("SideKi", Side.Pid.Ki);
    nh.getParam("SideKd", Side.Pid.Kd);
    nh.getParam("SideIntegralWindupGuard", Side.Pid.IntegralWindupGuard);
    nh.getParam("PidDeltaT", Side.Pid.DeltaT);

    nh.getParam("AltReferences", Altitude.References);
    nh.getParam("AltKp", Altitude.Pid.Kp);
    nh.getParam("AltKi", Altitude.Pid.Ki);
    nh.getParam("AltKd", Altitude.Pid.Kd);
    nh.getParam("AltIntegralWindupGuard", Altitude.Pid.IntegralWindupGuard);
    nh.getParam("PidDeltaT", Altitude.Pid.DeltaT);

    Yaw.Current = 0.0;
    Yaw.Pid.Error = 0.0;
    Yaw.Pid.LastError = 0.0;
    Yaw.Pid.Integral = 0.0;
    Yaw.Pid.Derivative = 0.0;
    Yaw.Pid.FirstDerivative = true;
    Yaw.Pid.Output = 0.0;
    Yaw.Pid.Rt.setEulerYPR(0, 0, 0);
    Yaw.Pid.Rd.setEulerYPR(0, 0, 0);

    Side.Current = 0.0;
    Side.Pid.Error = 0.0;
    Side.Pid.LastError = 0.0;
    Side.Pid.Integral = 0.0;
    Side.Pid.Derivative = 0.0;
    Side.Pid.FirstDerivative = true;
    Side.Pid.Output = 0.0;
    Side.Pid.Rt.setEulerYPR(0, 0, 0);
    Side.Pid.Rd.setEulerYPR(0, 0, 0);

    Front.Current = 0.0;
    Front.Pid.Error = 0.0;
    Front.Pid.LastError = 0.0;
    Front.Pid.Integral = 0.0;
    Front.Pid.Derivative = 0.0;
    Front.Pid.FirstDerivative = true;
    Front.Pid.Output = 0.0;
    Front.Pid.Rt.setEulerYPR(0, 0, 0);
    Front.Pid.Rd.setEulerYPR(0, 0, 0);

    Altitude.Current = 0.0;
    Altitude.Pid.Error = 0.0;
    Altitude.Pid.LastError = 0.0;
    Altitude.Pid.Integral = 0.0;
    Altitude.Pid.Derivative = 0.0;
    Altitude.Pid.FirstDerivative = true;
    Altitude.Pid.Output = 0.0;
    Altitude.Pid.Rt.setEulerYPR(0, 0, 0);
    Altitude.Pid.Rd.setEulerYPR(0, 0, 0);

    AutoControl = 0;
    MovState = 0;
    MovIndex = 0;
    MsgStr = "CONTROLLER ON";

    //------------- tum_simulator (Gazebo) ---------------------------
    nh.getParam("GazeboModelID", GazeboModelID);

    //------------- Neural Networks ---------------------------
    nh.getParam("PoseNet_FilterSize", PoseNetFilterSize);

    nh.getParam("ObjLoc_SideThr", ObjLocMsg.SideThrPixel);
    nh.getParam("ObjLoc_FrontThr", ObjLocMsg.FrontThrPixel);
    nh.getParam("ObjLoc_AltThr", ObjLocMsg.AltThr);
    nh.getParam("ObjLoc_YawThr", ObjLocMsg.YawThr);
    nh.getParam("ObjLoc_ScoreThr", ObjLocMsg.ScoreThr);
    nh.getParam("ObjLoc_ImgW", ObjLocMsg.ImgW);
    nh.getParam("ObjLoc_ImgH", ObjLocMsg.ImgH);

    // normalize thresholds for PID
    ObjLocMsg.SideThr = ObjLocMsg.SideThrPixel / ObjLocMsg.ImgW;
    ObjLocMsg.FrontThr = ObjLocMsg.FrontThrPixel / ObjLocMsg.ImgH;

    // Object Localization CNN bucket interface
    ObjLocMsg.Xc = 0;
    ObjLocMsg.Yc = 0;
    ObjLocMsg.W = 0;
    ObjLocMsg.H = 0;
    ObjLocMsg.Q.setValue(0,0,0,1); // avoid NaN in YawFiltered
    ObjLocMsg.SF = 0;
    ObjLocMsg.Score = 0;
    ObjLocMsg.XcFiltered = 0;
    ObjLocMsg.YcFiltered = 0;
    ObjLocMsg.YawFiltered = 0;
    ObjLocMsg.SFFiltered = 0;
    ObjLocMsg.QwFiltered = 0;
    ObjLocMsg.QxFiltered = 0;
    ObjLocMsg.QyFiltered = 0;
    ObjLocMsg.QzFiltered = 0;
    
    // Object Localization CNN grasper interface, coordinates only
    ObjGrasperMsg.ScoreThr = ObjLocMsg.ScoreThr;
    ObjGrasperMsg.ImgW = ObjLocMsg.ImgW;
    ObjGrasperMsg.ImgH = ObjLocMsg.ImgH;
    ObjGrasperMsg.Xc = 0;
    ObjGrasperMsg.Yc = 0;
    ObjGrasperMsg.W = 0;
    ObjGrasperMsg.H = 0;
    ObjGrasperMsg.Score = 0;
    ObjGrasperMsg.XcFiltered = 0;
    ObjGrasperMsg.YcFiltered = 0;
    Utils_KalmanInit(ObjLocMsg.XcKalman, 0, 0.0313, 0, 0.0313); // no units
    Utils_KalmanInit(ObjLocMsg.YcKalman, 0, 0.0556, 0, 0.0556); // no units

    // Metrics for machine learning
    Metrics.AccuracyTotal = 300;
    Metrics.AccuracyFail = 0;
    
    
    RollIntegral = 0.0;
    PitchIntegral = 0.0;
    UseBias = false;
    ControlSEnable = true;
    YawReached = false;

    HoveringOffCounter = 0;
    HoveringCounter = 0;
    HoveringSteps = 30;
    HoveringRoll = false;
    HoveringPitch = false;
    HoveringSignal = true;
    HoveringEnable = true;

    LiftEnable = false;
    LiftSteps = 75;
    LiftCounter = LiftSteps;
    LiftPositiveCounter = 0;
    LiftState = 0;

    ControlBIEnable = false;
    ControlBIPidEnable = false;
    ControlBIBangEnable = false;
    ControlBIBangCounter = 0;
    ControlBIBangState = 0;
    ControlBIBangSideOut = 0.0;
    ControlBIBangFrontOut = 0.0;
    ControlBIMaxCounter = 90;
    ControlBICounter = 0;
    ControlBIAcc = 0.0;
    ControlBISpeed = 0.3;
}


ControlFrontal::~ControlFrontal()
{
}


void ControlFrontal::Image_Callback4(const sensor_msgs::ImageConstPtr &msg) {
    try {
        MavImgCvBridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    MavImg = MavImgCvBridge->image;

    // Center-Crop image from Bebop 2 (856x480)->(640x360)
    Utils_ImageCenterCrop(MavImg, 640, 360);

    //std::cout << "Alitude.Current: " << Altitude.Current << std::endl;

//    Metrics.TimeEnd = clock();
//    std::cout << double(Metrics.TimeEnd - Metrics.TimeIni)/CLOCKS_PER_SEC << std::endl;
//    Metrics.TimeIni = Metrics.TimeEnd;


//    if(AutoControl == 1)
//    {
//        if(ObjLocMsg.Score > ObjLocMsg.ScoreThr)
//        {   // add object region and top view yaw orientation
//            ObjLoc_DrawBucketWithYaw(MavImg);
//            ostringstream strYaw;
//            strYaw << ObjLocMsg.Yaw * 180/M_PI;
//            cv::putText(MavImg, " Yaw: " + strYaw.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);

//            // draw intersection threshold
//            cv::circle(MavImg, cv::Point(ObjLocMsg.XcPixel, ObjLocMsg.YcPixel), 70, cv::Scalar(255,255,0), 1);
//        }
//        if(ObjGrasperMsg.Score > ObjGrasperMsg.ScoreThr)
//        {
//            ObjLoc_DrawGrasper(MavImg);
//        }
//    }


//    // draw threshold lines
//    // horizontal lines
//    double imgHCenter = ObjLocMsg.ImgH/2;
//    int tmp = int(imgHCenter - ObjLocMsg.FrontThrPixel);
//    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
//    tmp = int(imgHCenter + ObjLocMsg.FrontThrPixel);
//    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
//    // vertical lines
//    double imgWCenter = ObjLocMsg.ImgW/2;
//    tmp = int(imgWCenter - ObjLocMsg.SideThrPixel - 0);
//    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
//    tmp = int(imgWCenter + ObjLocMsg.SideThrPixel - 0);
//    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);

    // draw intersection threshold
    //cv::circle(MavImg, cv::Point(int(imgWCenter), int(imgHCenter)), 70, cv::Scalar(255,255,0), 1);


    ControlHoveringSignal();
    ControlByIntersection3();
    ControlLift2();

    imshow("DroneImage", MavImg);
    cv::waitKey(1);
}


void ControlFrontal::Image_Callback3(const sensor_msgs::ImageConstPtr &msg) {
    try {
        MavImgCvBridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    MavImg = MavImgCvBridge->image;

    // Center-Crop image from Bebop 2 (856x480)->(640x360)
    Utils_ImageCenterCrop(MavImg, 640, 360);

    //std::cout << "Alitude.Current: " << Altitude.Current << std::endl;

//    Metrics.TimeEnd = clock();
//    std::cout << double(Metrics.TimeEnd - Metrics.TimeIni)/CLOCKS_PER_SEC << std::endl;
//    Metrics.TimeIni = Metrics.TimeEnd;


//    if(AutoControl == 1) {
//        if(ObjLocMsg.Score > ObjLocMsg.ScoreThr)
//        {   // add object region and top view yaw orientation
//            ObjLoc_DrawBucket(MavImg);
//            // draw intersection threshold
//            cv::circle(MavImg, cv::Point(ObjLocMsg.XcPixel, ObjLocMsg.YcPixel), 70, cv::Scalar(255,255,0), 1);
//        }


//        if(ObjGrasperMsg.Score > ObjGrasperMsg.ScoreThr)
//        {
//            ObjLoc_DrawGrasper(MavImg);
//        }
//    }


//    // draw threshold lines
//    // horizontal lines
//    double imgHCenter = ObjLocMsg.ImgH/2;
//    int tmp = int(imgHCenter - ObjLocMsg.FrontThrPixel);
//    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
//    tmp = int(imgHCenter + ObjLocMsg.FrontThrPixel);
//    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
//    // vertical lines
//    double imgWCenter = ObjLocMsg.ImgW/2;
//    tmp = int(imgWCenter - ObjLocMsg.SideThrPixel - 0);
//    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
//    tmp = int(imgWCenter + ObjLocMsg.SideThrPixel - 0);
//    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);

    // draw intersection threshold
    //cv::circle(MavImg, cv::Point(int(imgWCenter), int(imgHCenter)), 70, cv::Scalar(255,255,0), 1);

    //ControlByIntersection();
    ControlByIntersection2();

    ControlLift2();

//    imshow("DroneImage", MavImg);
//    cv::waitKey(1);
}


void ControlFrontal::Image_Callback2(const sensor_msgs::ImageConstPtr &msg) {
    try {
        MavImgCvBridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    MavImg = MavImgCvBridge->image;

    // Center-Crop image from Bebop 2 (856x480)->(640x360)
    Utils_ImageCenterCrop(MavImg, 640, 360);

    //std::cout << "Alitude.Current: " << Altitude.Current << std::endl;

    PID_Control6B();

    ControlLift();
}


void ControlFrontal::Image_Callback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        MavImgCvBridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    MavImg = MavImgCvBridge->image;

    // Center-Crop image from Bebop 2 (856x480)->(640x360)
    Utils_ImageCenterCrop(MavImg, 640, 360);

    ObjLoc_DrawBucketWithYaw(MavImg);

    if(AutoControl == 0) {
        cv::putText(MavImg, "CONTROLLER OFF", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 0, 0), 2, 2);
        //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
        MovState = MOV4_SIDE;
        MovIndex = 0;
        MsgStr = "CONTROLLER ON";
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }

    ostringstream strYaw, strHeight;
    strHeight << ObjLocMsg.Altitude;
    strYaw << ObjLocMsg.Yaw * 180/M_PI;

    cv::putText(MavImg, " Yaw: " + strYaw.str() + " " + " Height: " + strHeight.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
    //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
    cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);

    // draw threshold lines
    // horizontal lines
    int tmp = int(ObjLocMsg.ImgH/2 - 25);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    tmp = int(ObjLocMsg.ImgH/2 + 25);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    // vertical lines
    tmp = int(ObjLocMsg.ImgW/2 - 30);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    tmp = int(ObjLocMsg.ImgW/2 + 30);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    
    // slow down area
    //cv::rectangle(MavImg, cv::Point(int(ObjLocMsg.ImgW/2 - 100), int(ObjLocMsg.ImgH/2 - 100)), cv::Point(int(ObjLocMsg.ImgW/2 + 100), int(ObjLocMsg.ImgH/2 + 100)), cv::Scalar(255, 0, 0), 1);
    //cv::rectangle(MavImg, cv::Point(int(ObjLocMsg.ImgW/2 - 65), int(ObjLocMsg.ImgH/2 - 65)), cv::Point(int(ObjLocMsg.ImgW/2 + 65), int(ObjLocMsg.ImgH/2 + 65)), cv::Scalar(255, 0, 0), 1);

    ControlS();
    
    imshow("DroneImage", MavImg);
    cv::waitKey(1);
}


void ControlFrontal::ControlS(void)
{
//    ObjLoc_DrawObject(MavImg, 40, 20);

    //********** ALTITUDE

    double errorAlt = 1.0 - Altitude.Current;
    double signalAlt = 0.5 * errorAlt;

//    std::cout << "===== P Altitude =====" << std::endl;
//    std::cout << "Altitude.Current " << Altitude.Current << " errorAlt " << errorAlt << " signalAlt " << signalAlt <<  std::endl;


    if(signalAlt > 0.5) signalAlt = 0.5;
    if(signalAlt < -0.5) signalAlt = -0.5;

    // positive -> moves up, always published¡
    MavCommand.linear.z = signalAlt;

    // hovering
    if(ObjLocMsg.Score < ObjLocMsg.ScoreThr || HoveringCounter > 0)
    {
        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;

        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;

        MavCommandPub.publish(MavCommand);

        if(ObjLocMsg.Score < ObjLocMsg.ScoreThr)
            std::cout << "No object! so we stay in hovering!!  ObjLocMsg.Score: " << ObjLocMsg.Score << std::endl;

        if(HoveringCounter > 0)
        {
            HoveringCounter--;
            std::cout << "We are in hovering regions: HoveringCounter: " << HoveringCounter << std::endl;
        }
        return;
    }

    // we allow hovering only for a fixed amount of time
    if(HoveringOffCounter > 0) {
        HoveringOffCounter--;
        std::cout << "HOVERING OFF " << HoveringOffCounter << std::endl;
    }

    HoveringRoll = false;
    HoveringPitch = false;


    //********** SIDE

    double posPx = ObjLocMsg.ImgW*(ObjLocMsg.Xc - 0.5);
    double errorRoll = 1.0 - 2*ObjLocMsg.Xc;

//    std::cout << "ObjLocMsg.Xc " << ObjLocMsg.Xc << std::endl;
//    std::cout << "posPx "<< posPx << std::endl;

    if(abs(posPx) < 30.0)
    {
        HoveringRoll = true;
//        std::cout << "@@@@@ HOVERING X @@@@@" << std::endl;

        RollIntegral = 0.0;

        std::cout << "signalRoll " << 0 << std::endl;

        MavCommand.linear.y = 0;
        //HoveringCounter = HoveringSteps;
    }
    else if(abs(posPx) < 65.0)
    {
//        std::cout << "+++++ PIntegral X +++++" << std::endl;

        RollIntegral += errorRoll;
        double bias = 0.03;
        if(errorRoll < 0) bias = -0.03;
        if(UseBias == false) bias = 0;

        double signalRoll = 0.008*errorRoll + 0.01*RollIntegral*0.03333 + bias;

//        std::cout << "errorRoll " << errorRoll << std::endl;
        std::cout << "signalRoll " << signalRoll << std::endl;

        MavCommand.linear.y = signalRoll;
    }
    else if(abs(posPx) < 100.0)
    {
//        std::cout << "***** PIntegral X + HOV *****" << std::endl;

        RollIntegral = 0;

        double signalRoll = 0.008*errorRoll; // + 0.01*RollIntegral*0.03333;

//        std::cout << "errorRoll " << errorRoll << std::endl;
        std::cout << "signalRoll " << signalRoll << std::endl;

        MavCommand.linear.y = signalRoll;
    }
    else
    {
        RollIntegral = 0.0;

        double signalRoll = 0.01*errorRoll; // + 0.1*deltaError*30;

        MavCommand.linear.y = signalRoll;

//        std::cout << "===== P X only =====" << std::endl;
//        std::cout << "errorRoll " << errorRoll << std::endl;
        std::cout << "signalRoll " << signalRoll << std::endl;
    }


    //********** FRONT

    double posPy = ObjLocMsg.ImgH*(0.5 - ObjLocMsg.Yc);
    double errorPitch = 1.0 - 2*ObjLocMsg.Yc;

//    std::cout << "ObjLocMsg.Yc " << ObjLocMsg.Yc << std::endl;
//    std::cout << "posPy "<< posPy << std::endl;

    if(abs(posPy) < 25.0)
    {
        HoveringPitch = true;
//        std::cout << "@@@@@ HOVERING Y @@@@@" << std::endl;

        PitchIntegral = 0.0;

        MavCommand.linear.x = 0;
        //HoveringCounter = HoveringSteps;
    }
    else if(abs(posPy) < 65.0)
    {
//        std::cout << "+++++ PIntegral Y +++++" << std::endl;

        PitchIntegral += errorPitch;
        double bias = 0.015;
        if(errorPitch < 0) bias = -0.015;
        if(UseBias == false) bias = 0;

        double signalPitch = 0.008*errorPitch + 0.01*PitchIntegral*0.03333 + bias;

//        std::cout << "errorPitch " << errorPitch << std::endl;
//        std::cout << "signalPitch " << signalPitch << std::endl;

        MavCommand.linear.x = signalPitch;
    }
    else if(abs(posPy) < 100.0)
    {
//        std::cout << "***** PIntegral Y + HOV *****" << std::endl;

        PitchIntegral = 0;

        double signalPitch = 0.01*errorPitch ; //+ 0.01*PitchIntegral*0.03333;

//        std::cout << "errorPitch " << errorPitch << std::endl;
//        std::cout << "signalPitch " << signalPitch << std::endl;

        MavCommand.linear.x = signalPitch;
    }
    else
    {
        PitchIntegral = 0.0;

        double signalPitch = 0.01*errorPitch; // + 0.05*deltaError*30;

        MavCommand.linear.x = signalPitch;

//        std::cout << "===== P only Y =====" << std::endl;
//        std::cout << "errorPitch " << errorPitch << std::endl;
//        std::cout << "signalPitch " << signalPitch << std::endl;
    }


    //********** YAW

    if(HoveringRoll && HoveringPitch)
    {
        UseBias = true;

        double errorYaw = ObjLocMsg.Yaw*180/M_PI - Yaw.References[MovIndex];
        double signalYaw = 0.0003 * errorYaw;

        if(signalYaw > 0.05) signalYaw = 0.05;
        if(signalYaw < -0.05) signalYaw = -0.05;

        // positive -> moves up, always published¡
        MavCommand.angular.z = signalYaw;

        if(abs(errorYaw) < 5)
        {
            MsgStr = "REFERENCE REACHED";
        }
    }
    else
    {
        MavCommand.angular.z = 0;
        MsgStr = "CONTROLLER ON";
    }



    if(HoveringOffCounter == 0 && (HoveringRoll && HoveringPitch))
    {
        HoveringCounter = HoveringSteps;
        HoveringOffCounter = 2*HoveringSteps;
    }

    MavCommand.angular.x = 0;
    MavCommand.angular.y = 0;

    MavCommandPub.publish(MavCommand);
}


void ControlFrontal::ControlHoveringSignal(void) {
// Creates an astable oscilator signal OnOffOnOff...

    if(HoveringEnable == false)
    {
        HoveringCounter = HoveringSteps;
        HoveringSignal = true;
        return;
    }
    
    if(HoveringCounter > 0)
        HoveringCounter--;
    else if(HoveringCounter == 0)
    {
        if(HoveringSignal == true)
        {
            HoveringSignal = false;
            HoveringCounter = 3*HoveringSteps;
        }
        else
        {
            HoveringSignal = true;
            HoveringCounter = HoveringSteps;
        }
    }
}


void ControlFrontal::ControlByIntersection3(void) {
// First we orient the vehicle with the handle at 90 degrees
// while keeping the bucket at the center of the image's reference frame
// Then whe keep the vehicle at the center of the Y axis and use
// Bang-bang only for the X axis.
// We always check the intersection and lift if the condition is met.
// Version for BEBOP 2.
// Individual control for  altitude, yaw, side and front velocities using
// ObjectLocalization CNN for BUCKET detection and SDD for GRASPER detection.
// The NN outputs:
// Object's orientation (quaternion)
// Object's center (x,y) (pixels)
// Object's height, width (pixels)
// Object's confidence [0,1]
// Object's Scale Factor [-1,1], 0 -> 0.5m, Not used here¡

    if(ObjLocMsg.Score > ObjLocMsg.ScoreThr)
    {   // add object region and top view yaw orientation
        ObjLoc_DrawBucketWithYaw(MavImg);
//        ostringstream strYaw;
//        strYaw << ObjLocMsg.Yaw * 180/M_PI;
//        cv::putText(MavImg, " Yaw: " + strYaw.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
        
        // draw intersection threshold
        cv::circle(MavImg, cv::Point(ObjLocMsg.XcPixel, ObjLocMsg.YcPixel), 70, cv::Scalar(255,255,0), 1);
    }

    if(ObjGrasperMsg.Score > ObjGrasperMsg.ScoreThr)
    {
        ObjLoc_DrawGrasper(MavImg);
    }

    if(AutoControl == 0) {
        cv::putText(MavImg, "CONTROLLER OFF", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 0, 0), 2, 2);
        //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
        MovIndex = 0;
        MsgStr = "CONTROLLER ON";
        ControlBIEnable = true;
        ControlBIPidEnable = false;
        ControlBIBangEnable = false;
        HoveringEnable = false;
        LiftEnable = false;
        LiftState = 0;
        LiftCounter = LiftSteps;
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }

    // disable PID control in lifting stage
    if(ControlBIEnable == false)
    {
        cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }


    ostringstream strHeight, strYaw;
    //strHeight << ObjLocMsg.Altitude;
    strHeight << Altitude.Current;
    strYaw << ObjLocMsg.YawFiltered * 180/M_PI;

    cv::putText(MavImg, " Yaw: " + strYaw.str() + " " + " Height: " + strHeight.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
    //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
    cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);

    // draw threshold lines
    // horizontal lines
    double imgHCenter = ObjLocMsg.ImgH/2;
    int tmp = int(imgHCenter - ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    tmp = int(imgHCenter + ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);

//    tmp = int(imgHCenter - 140);
//    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
//    tmp = int(imgHCenter + 140);
//    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);

    // vertical lines
    double imgWCenter = ObjLocMsg.ImgW/2;
    tmp = int(imgWCenter - ObjLocMsg.SideThrPixel - 0);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    tmp = int(imgWCenter + ObjLocMsg.SideThrPixel - 0);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);

    // bang-bang border
    tmp = int(imgWCenter - 140);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    tmp = int(imgWCenter + 140);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);

    // draw intersection threshold
    //cv::circle(MavImg, cv::Point(int(imgWCenter), int(imgHCenter)), 70, cv::Scalar(255,255,0), 1);

    imshow("DroneImage", MavImg);
    cv::waitKey(1);



    //********** ALTITUDE

    bool altReached = false;
    double deltaError;
    // Moves in meters, refs must be > 0
    //Altitude.Pid.Error = Altitude.References[MovIndex] - ObjLocMsg.Altitude;
    Altitude.Pid.Error = Altitude.References[MovIndex] - Altitude.Current;

    // integral term
    Altitude.Pid.Integral += Altitude.Pid.Error * Altitude.Pid.DeltaT;
    // clamp windup
    if(Altitude.Pid.Integral < -Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = -Altitude.Pid.IntegralWindupGuard;
    else if(Altitude.Pid.Integral > Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = Altitude.Pid.IntegralWindupGuard;

    // derivative term
    deltaError = Altitude.Pid.Error - Altitude.Pid.LastError;
    Altitude.Pid.Derivative = deltaError / Altitude.Pid.DeltaT;
    Altitude.Pid.LastError = Altitude.Pid.Error;

    //Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error);

    if(abs(Altitude.Pid.Error) < 5*ObjLocMsg.AltThr)
    {
        Altitude.Pid.Output = 0;
        Altitude.Pid.Integral = 0;
        Altitude.Pid.LastError = Altitude.Pid.Error;
        altReached = true;
    }
    else {
        Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error) +
                (Altitude.Pid.Ki * Altitude.Pid.Integral) +
                (Altitude.Pid.Kd * Altitude.Pid.Derivative);
    }

    if(Altitude.Pid.Output > 0.5) Altitude.Pid.Output = 0.5;
    if(Altitude.Pid.Output < -0.5) Altitude.Pid.Output = -0.5;

    // positive -> moves up
    MavCommand.linear.z = Altitude.Pid.Output;


    //********* METRICS

    if(Metrics.AccuracyTotal > 0)
    {
        Metrics.AccuracyTotal--;

        //if(ObjLocMsg.Score < ObjLocMsg.ScoreThr)
        if(ObjGrasperMsg.Score < ObjGrasperMsg.ScoreThr)
        {
            Metrics.AccuracyFail++;
        }
    }
    else if(Metrics.AccuracyTotal == 0)
    {
        ROS_INFO_STREAM("++++++ METRICS ACCURACY, Fail: " << Metrics.AccuracyFail);
        // deactivate printing
        Metrics.AccuracyTotal = -1;
    }


    //********** HOVERING

    if(ObjLocMsg.Score < ObjLocMsg.ScoreThr) {
        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;

        Side.Pid.Integral = 0;
        Side.Pid.FirstDerivative = true;
        Front.Pid.Integral = 0;
        Front.Pid.FirstDerivative = true;
        Yaw.Pid.Integral = 0;
        Yaw.Pid.FirstDerivative = true;

        ControlBIPidEnable = false;
        ControlBIBangEnable = false;
        ControlBIBangState = 0;

        ROS_INFO_STREAM("NO OBJECT, HOVERING");

        MavCommandPub.publish(MavCommand);
        return;
    }

    //if(HoveringCounter > 0)
    if(HoveringEnable && HoveringSignal == true)
    {
        //HoveringCounter--;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;

        //ROS_INFO_STREAM("We are in hovering regions. HoveringCounter: " << HoveringCounter);

        MavCommandPub.publish(MavCommand);
        return;
    }

    // we allow hovering only for a fixed amount of time
//    if(HoveringOffCounter > 0)
//    {
//        HoveringOffCounter--;
        //ROS_INFO_STREAM("HOVERING OFF " << HoveringOffCounter);
//    }



    //********** INTERSECTION, distance between bucket and grasper

    int distance;
    distance = int( sqrt( double((ObjGrasperMsg.XcPixel - ObjLocMsg.XcPixel)*(ObjGrasperMsg.XcPixel - ObjLocMsg.XcPixel)) + double((ObjGrasperMsg.YcPixel - ObjLocMsg.YcPixel)*(ObjGrasperMsg.YcPixel - ObjLocMsg.YcPixel)) ) );

    if(distance < 70)
    {
        if(ObjGrasperMsg.Score >= ObjGrasperMsg.ScoreThr)
            ControlBICounter++;
    }

    if(ControlBIMaxCounter > 0)
    {
        ControlBIMaxCounter--;
    }
    else if(ControlBIMaxCounter == 0)
    {
        ControlBIMaxCounter = 90;
        ControlBIAcc = double(ControlBICounter) / double(ControlBIMaxCounter);
        ROS_INFO_STREAM("Intersection Counter: " << ControlBICounter << " Intersection Accuracy: " << ControlBIAcc);
        ControlBICounter = 0;

        if(ControlBIAcc > 0.5)
        {
            LiftEnable = true;
            ControlBIEnable = false;
            ROS_INFO_STREAM("****** READY TO LIFT...");

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            //MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            //MavCommand.linear.z = 0;

            MavCommandPub.publish(MavCommand);
            return;
        }
    }



    //********** FRONT
    
    bool frontReached = false;

    //Front.Pid.Error = Front.References[MovIndex] - ObjLocMsg.YcPixel;
    //Front.Pid.Error = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
    double newError = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
    Utils_MeanFilter(newError, Front.Pid.Error, PoseNetFilterSize);

    double error;
    error = Front.Pid.Error;

    if(abs(Front.Pid.Error) < ObjLocMsg.FrontThr)
    {
        Front.Pid.Output = 0;
        Front.Pid.Integral = 0;
        Front.Pid.LastError = error;

        frontReached = true;
        //ROS_INFO_STREAM("______ FRONT REACHED...");
    }
    else {
//            error = sqrt(abs(Front.Pid.Error) - ObjLocMsg.FrontThr);
//            if(Front.Pid.Error < 0)
//                error = -error;

        // integral term
        Front.Pid.Integral += error * Front.Pid.DeltaT;
        // clamp windup
        if(Front.Pid.Integral < -Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = -Front.Pid.IntegralWindupGuard;
        else if(Front.Pid.Integral > Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = Front.Pid.IntegralWindupGuard;

        // derivative term
        deltaError = error - Front.Pid.LastError;
        Front.Pid.Derivative = deltaError / Front.Pid.DeltaT;
        Front.Pid.LastError = error;
//        if(Front.Pid.FirstDerivative == true)
//        {
//            Front.Pid.Derivative = 0;
//            Front.Pid.FirstDerivative = false;
//        }


        Front.Pid.Output = (Front.Pid.Kp * error) +
                (Front.Pid.Ki * Front.Pid.Integral) +
                (Front.Pid.Kd * Front.Pid.Derivative);
    }

    //positive -> moves front
    MavCommand.linear.x = Front.Pid.Output;
    //ROS_INFO_STREAM("Pre " << ObjLocMsg.Yc << " Error " << newError << " U " << Front.Pid.Output);



    //*********** Go to CENTER or use BANG-BANG

    //MavCommand.linear.x = 0;
    MavCommand.linear.y = 0;
    //MavCommand.linear.z = 0;

    if(ControlBIPidEnable == false)
    {
        if(YawReached == false)
        {
            ControlBIPidEnable = true;
            ControlBIBangEnable = false;
            ControlBIBangState = 0;
        }
        else
        {
            //int imgHCenter = int(ObjLocMsg.ImgH/2);
            int imgWCenter = int(ObjLocMsg.ImgW/2);

            int xDistance = abs(imgWCenter - ObjLocMsg.XcPixel);
            //int yDistance = abs(imgHCenter - ObjLocMsg.YcPixel);

            //if(xDistance > 140 || yDistance > 140)
            if(xDistance > 140)
            {
                ControlBIPidEnable = true;
                ControlBIBangEnable = false;
                ControlBIBangState = 0;
                ROS_INFO_STREAM("###### PID ENABLE...");
            }
            else
            {
                ControlBIPidEnable = false;
                ControlBIBangEnable = true;
                //ROS_INFO_STREAM("++++++ BANG ENABLE...");
            }
        }
    }



    if(ControlBIPidEnable)
    {
        //********** SIDE
        
        bool sideReached = false;

        //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
        //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        newError = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

        //double error;
        error = Side.Pid.Error;

        if(abs(Side.Pid.Error) < ObjLocMsg.SideThr)
        {
            Side.Pid.Output = 0;
            Side.Pid.Integral = 0;
            Side.Pid.LastError = error;

            sideReached = true;
            //ROS_INFO_STREAM("______ SIDE REACHED...");
        }
        else {
    //            error = sqrt(abs(Side.Pid.Error) - ObjLocMsg.SideThr);
    //            if(Side.Pid.Error < 0)
    //                error = -error;

            // integral term
            Side.Pid.Integral += error * Side.Pid.DeltaT;
            // clamp windup
            if(Side.Pid.Integral < -Side.Pid.IntegralWindupGuard)
                Side.Pid.Integral = -Side.Pid.IntegralWindupGuard;
            else if(Side.Pid.Integral > Side.Pid.IntegralWindupGuard)
                Side.Pid.Integral = Side.Pid.IntegralWindupGuard;

            // derivative term
            double deltaError = error - Side.Pid.LastError;
            Side.Pid.Derivative = deltaError / Side.Pid.DeltaT;
            Side.Pid.LastError = error;
    //        if(Side.Pid.FirstDerivative == true)
    //        {
    //            Side.Pid.Derivative = 0;
    //            Side.Pid.FirstDerivative = false;
    //        }


            Side.Pid.Output = (Side.Pid.Kp * error) +
                    (Side.Pid.Ki * Side.Pid.Integral) +
                    (Side.Pid.Kd * Side.Pid.Derivative);
        }

        //positive -> moves left
        MavCommand.linear.y = Side.Pid.Output;
        //ROS_INFO_STREAM("Pre " << ObjLocMsg.Xc << " Error " << newError << " U " << Side.Pid.Output);



        //if(HoveringOffCounter == 0 && sideReached && frontReached && HoveringEnable)
        if(YawReached == false && sideReached && frontReached && HoveringEnable == false)
        {
            // start hovering
            //HoveringCounter = HoveringSteps;
            //HoveringOffCounter = 2*HoveringSteps;
            HoveringEnable = true;

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            //MavCommand.linear.z = 0;
        }
    //    else
//        if(sideReached && frontReached && altReached)
        else if(YawReached == false && sideReached && frontReached)
        {
            //********** YAW


            // Referencia en grados
    //#ifdef BEBOP2
            //Yaw.Pid.Error = Yaw.References[MovIndex] - ObjLocMsg.Yaw*180/M_PI;
    //#else
            Yaw.Pid.Error = ObjLocMsg.Yaw*180/M_PI - Yaw.References[MovIndex]; // tum_simulator
    //#endif

            // integral term
            Yaw.Pid.Integral += Yaw.Pid.Error * Yaw.Pid.DeltaT;
            // clamp windup
            if(Yaw.Pid.Integral < -Yaw.Pid.IntegralWindupGuard)
                Yaw.Pid.Integral = -Yaw.Pid.IntegralWindupGuard;
            else if(Yaw.Pid.Integral > Yaw.Pid.IntegralWindupGuard)
                Yaw.Pid.Integral = Yaw.Pid.IntegralWindupGuard;

            // derivative term
            deltaError = Yaw.Pid.Error - Yaw.Pid.LastError;
            Yaw.Pid.Derivative = deltaError / Yaw.Pid.DeltaT;
            Yaw.Pid.LastError = Yaw.Pid.Error;


            if(abs(Yaw.Pid.Error) < ObjLocMsg.YawThr)
            {
                Yaw.Pid.Output = 0;
                Yaw.Pid.Integral = 0;
                Yaw.Pid.LastError = Yaw.Pid.Error;

                YawReached = true;
                ROS_INFO_STREAM("****** YAW REACHED ******");
            }
            else {
                Yaw.Pid.Output = (Yaw.Pid.Kp * Yaw.Pid.Error) +
                        (Yaw.Pid.Ki * Yaw.Pid.Integral) +
                        (Yaw.Pid.Kd * Yaw.Pid.Derivative);
            }

            // positive -> turns right?
            MavCommand.angular.z = Yaw.Pid.Output;
        }
        else if(YawReached && sideReached && frontReached)
        {
            HoveringEnable = false;
            ControlBIBangEnable = true;
            ControlBIBangState = 0;
            ControlBIPidEnable = false;

            ROS_INFO_STREAM("Center Reached");
            ROS_INFO_STREAM("++++++ BANG ENABLE");
            //MsgStr = "CENTER REACHED";
            
            MavCommand.angular.z = 0;
        }
        else
        {
            MavCommand.angular.z = 0;
        }
    }



    if(ControlBIBangEnable)
    {
        if(ObjGrasperMsg.Score < ObjGrasperMsg.ScoreThr)
        {
            ControlBIBangState = 3; // go to default to abort
            //MavCommand.linear.x = 0;
            //MavCommand.linear.y = 0;
        }

        switch(ControlBIBangState)
        {
        case 0: {
            ControlBIBangState = 1;
            ControlBIBangCounter = 3;

            //********** SIDE

            //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
            //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
            int errorPixel = ObjLocMsg.XcPixel - ObjGrasperMsg.XcPixel;
            //Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

            if(abs(errorPixel) < 20)
                ControlBIBangSideOut = 0;
            else if(errorPixel < 0)
                ControlBIBangSideOut = ControlBISpeed;
            else
                ControlBIBangSideOut = -ControlBISpeed;

            //positive -> moves left
            MavCommand.linear.y = ControlBIBangSideOut;

            //ROS_INFO_STREAM("Pre " << ObjLocMsg.Xc << " Error " << newError << " U " << Side.Pid.Output);
            ROS_INFO_STREAM("______ BANG STATE 0...");
        } break;
        case 1: {
            if(ControlBIBangCounter > 0)
            {
                ControlBIBangCounter--;
                ControlBIBangState = 1;
                MavCommand.linear.y = ControlBIBangSideOut;
                //MavCommand.linear.x = ControlBIBangFrontOut;
            }
            else
            {
                ControlBIBangState = 2;
                ControlBIBangCounter = 60;
                MavCommand.linear.y = 0;
                //MavCommand.linear.x = 0;
            }
        } break;
        case 2: {
            if(ControlBIBangCounter > 0)
            {
                ControlBIBangCounter--;
                ControlBIBangState = 2;
            }
            else
            {
                ControlBIBangState = 0;
            }

            MavCommand.linear.y = 0;
            //MavCommand.linear.x = 0;
        } break;
        default: {
            ControlBIBangState = 0;
            MavCommand.linear.y = 0;
            //MavCommand.linear.x = 0;
        } break;
        } // end switch(ControlBIBangState)
    }


    MavCommand.angular.x = 0;
    MavCommand.angular.y = 0;
    //MavCommand.angular.z = 0;

    // Update velocities
    MavCommandPub.publish(MavCommand);
    //ROS_INFO_STREAM("Ux " << MavCommand.linear.x << " Uy " << MavCommand.linear.y << " Uz " << MavCommand.linear.z << " Uwz " << MavCommand.angular.z);
}



void ControlFrontal::ControlByIntersection2(void) {
// We keep the bucket at the center of the Y axis. Bang-bang only for X axis.
// Version for BEBOP 2.
// Individual control for  altitude, side and front velocities using
// ObjectLocalization CNN for BUCKET detection and SDD for GRASPER detection.
// The NN outputs:
// Object's orientation (quaternion), Not used here¡
// Object's center (x,y) (pixels)
// Object's height, width (pixels)
// Object's confidence [0,1]
// Object's Scale Factor [-1,1], 0 -> 0.5m, Not used here¡

    if(ObjLocMsg.Score > ObjLocMsg.ScoreThr)
    {   // add object region, dont display yaw orientation
        ObjLoc_DrawBucket(MavImg);
        // draw intersection threshold
        cv::circle(MavImg, cv::Point(ObjLocMsg.XcPixel, ObjLocMsg.YcPixel), 70, cv::Scalar(255,255,0), 1);
    }

    if(ObjGrasperMsg.Score > ObjGrasperMsg.ScoreThr)
    {
        ObjLoc_DrawGrasper(MavImg);
    }

    if(AutoControl == 0) {
        cv::putText(MavImg, "CONTROLLER OFF", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 0, 0), 2, 2);
        //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
        MovIndex = 0;
        MsgStr = "CONTROLLER ON";
        ControlBIEnable = true;
        ControlBIPidEnable = false;
        ControlBIBangEnable = true;
        HoveringEnable = true;
        LiftEnable = false;
        LiftState = 0;
        LiftCounter = LiftSteps;
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }

    // disable PID control in lifting stage
    if(ControlBIEnable == false)
    {
        cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }


    ostringstream strHeight;
    //strHeight << ObjLocMsg.Altitude;
    strHeight << Altitude.Current;

    cv::putText(MavImg, "Height: " + strHeight.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
    //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
    cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);

    // draw threshold lines
    // horizontal lines
    double imgHCenter = ObjLocMsg.ImgH/2;
    int tmp = int(imgHCenter - ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    tmp = int(imgHCenter + ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);

//    tmp = int(imgHCenter - 140);
//    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
//    tmp = int(imgHCenter + 140);
//    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);

    // vertical lines
    double imgWCenter = ObjLocMsg.ImgW/2;
    tmp = int(imgWCenter - ObjLocMsg.SideThrPixel - 0);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    tmp = int(imgWCenter + ObjLocMsg.SideThrPixel - 0);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);

    tmp = int(imgWCenter - 140);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    tmp = int(imgWCenter + 140);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);

    // draw intersection threshold
    //cv::circle(MavImg, cv::Point(int(imgWCenter), int(imgHCenter)), 70, cv::Scalar(255,255,0), 1);

    imshow("DroneImage", MavImg);
    cv::waitKey(1);



    //********** ALTITUDE

    bool altReached = false;
    double deltaError;
    // Moves in meters, refs must be > 0
    //Altitude.Pid.Error = Altitude.References[MovIndex] - ObjLocMsg.Altitude;
    Altitude.Pid.Error = Altitude.References[MovIndex] - Altitude.Current;

    // integral term
    Altitude.Pid.Integral += Altitude.Pid.Error * Altitude.Pid.DeltaT;
    // clamp windup
    if(Altitude.Pid.Integral < -Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = -Altitude.Pid.IntegralWindupGuard;
    else if(Altitude.Pid.Integral > Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = Altitude.Pid.IntegralWindupGuard;

    // derivative term
    deltaError = Altitude.Pid.Error - Altitude.Pid.LastError;
    Altitude.Pid.Derivative = deltaError / Altitude.Pid.DeltaT;
    Altitude.Pid.LastError = Altitude.Pid.Error;

    //Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error);

    if(abs(Altitude.Pid.Error) < 5*ObjLocMsg.AltThr)
    {
        Altitude.Pid.Output = 0;
        Altitude.Pid.Integral = 0;
        Altitude.Pid.LastError = Altitude.Pid.Error;
        altReached = true;
    }
    else {
        Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error) +
                (Altitude.Pid.Ki * Altitude.Pid.Integral) +
                (Altitude.Pid.Kd * Altitude.Pid.Derivative);
    }

    if(Altitude.Pid.Output > 0.5) Altitude.Pid.Output = 0.5;
    if(Altitude.Pid.Output < -0.5) Altitude.Pid.Output = -0.5;

    // positive -> moves up
    MavCommand.linear.z = Altitude.Pid.Output;


    //********* METRICS

//    if(Metrics.AccuracyTotal > 0)
//    {
//        Metrics.AccuracyTotal--;

//        if(ObjLocMsg.Score < ObjLocMsg.ScoreThr)
//        {
//            Metrics.AccuracyFail++;
//        }
//    }
//    else if(Metrics.AccuracyTotal == 0)
//    {
//        ROS_INFO_STREAM("+++++++++++++++++++ ACCURACY, Fail: " << Metrics.AccuracyFail);
//        // deactivate printing
//        Metrics.AccuracyTotal = -1;
//    }


    //********** HOVERING

    if(ObjLocMsg.Score < ObjLocMsg.ScoreThr) {
        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;

        Side.Pid.Integral = 0;
        Side.Pid.FirstDerivative = true;
        Front.Pid.Integral = 0;
        Front.Pid.FirstDerivative = true;
        Yaw.Pid.Integral = 0;
        Yaw.Pid.FirstDerivative = true;

        ControlBIPidEnable = false;
        ControlBIBangEnable = false;
        ControlBIBangState = 0;

        ROS_INFO_STREAM("NO OBJECT, HOVERING");

        MavCommandPub.publish(MavCommand);
        return;
    }

    if(HoveringCounter > 0)
    {
        HoveringCounter--;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;

        //ROS_INFO_STREAM("We are in hovering regions. HoveringCounter: " << HoveringCounter);

        MavCommandPub.publish(MavCommand);
        return;
    }

    // we allow hovering only for a fixed amount of time
    if(HoveringOffCounter > 0)
    {
        HoveringOffCounter--;
        //ROS_INFO_STREAM("HOVERING OFF " << HoveringOffCounter);
    }



    //********** FRONT

    //Front.Pid.Error = Front.References[MovIndex] - ObjLocMsg.YcPixel;
    //Front.Pid.Error = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
    double newError = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
    Utils_MeanFilter(newError, Front.Pid.Error, PoseNetFilterSize);

    double error;
    error = Front.Pid.Error;

    if(abs(Front.Pid.Error) < ObjLocMsg.FrontThr)
    {
        Front.Pid.Output = 0;
        Front.Pid.Integral = 0;
        Front.Pid.LastError = error;

        //frontReached = true;
        //ROS_INFO_STREAM("______ FRONT REACHED...");
    }
    else {
//            error = sqrt(abs(Front.Pid.Error) - ObjLocMsg.FrontThr);
//            if(Front.Pid.Error < 0)
//                error = -error;

        // integral term
        Front.Pid.Integral += error * Front.Pid.DeltaT;
        // clamp windup
        if(Front.Pid.Integral < -Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = -Front.Pid.IntegralWindupGuard;
        else if(Front.Pid.Integral > Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = Front.Pid.IntegralWindupGuard;

        // derivative term
        deltaError = error - Front.Pid.LastError;
        Front.Pid.Derivative = deltaError / Front.Pid.DeltaT;
        Front.Pid.LastError = error;
//        if(Front.Pid.FirstDerivative == true)
//        {
//            Front.Pid.Derivative = 0;
//            Front.Pid.FirstDerivative = false;
//        }


        Front.Pid.Output = (Front.Pid.Kp * error) +
                (Front.Pid.Ki * Front.Pid.Integral) +
                (Front.Pid.Kd * Front.Pid.Derivative);
    }

    //positive -> moves front
    MavCommand.linear.x = Front.Pid.Output;
    //ROS_INFO_STREAM("Pre " << ObjLocMsg.Yc << " Error " << newError << " U " << Front.Pid.Output);



    //********** INTERSECTION, distance between bucket and grasper

    int distance;
    distance = int( sqrt( double((ObjGrasperMsg.XcPixel - ObjLocMsg.XcPixel)*(ObjGrasperMsg.XcPixel - ObjLocMsg.XcPixel)) + double((ObjGrasperMsg.YcPixel - ObjLocMsg.YcPixel)*(ObjGrasperMsg.YcPixel - ObjLocMsg.YcPixel)) ) );

    if(distance < 70)
    {
        if(ObjGrasperMsg.Score >= ObjGrasperMsg.ScoreThr)
            ControlBICounter++;
    }

    if(ControlBIMaxCounter > 0)
    {
        ControlBIMaxCounter--;
    }
    else if(ControlBIMaxCounter == 0)
    {
        ControlBIMaxCounter = 90;
        ControlBIAcc = double(ControlBICounter) / double(ControlBIMaxCounter);
        ROS_INFO_STREAM("Intersection Counter: " << ControlBICounter << " Intersection Accuracy: " << ControlBIAcc);
        ControlBICounter = 0;

        if(ControlBIAcc > 0.5)
        {
            LiftEnable = true;
            ControlBIEnable = false;
            ROS_INFO_STREAM("****** READY TO LIFT...");

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            //MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            //MavCommand.linear.z = 0;

            MavCommandPub.publish(MavCommand);
            return;
        }
    }



    //*********** Go to CENTER or use BANG-BANG

    //MavCommand.linear.x = 0;
    MavCommand.linear.y = 0;
    //MavCommand.linear.z = 0;

    if(ControlBIPidEnable == false)
    {
        //int imgHCenter = int(ObjLocMsg.ImgH/2);
        int imgWCenter = int(ObjLocMsg.ImgW/2);

        int xDistance = abs(imgWCenter - ObjLocMsg.XcPixel);
        //int yDistance = abs(imgHCenter - ObjLocMsg.YcPixel);

        //if(xDistance > 140 || yDistance > 140)
        if(xDistance > 140)
        {
            ControlBIPidEnable = true;
            ControlBIBangEnable = false;
            ControlBIBangState = 0;
            ROS_INFO_STREAM("###### PID ENABLE...");
        }
        else
        {
            ControlBIPidEnable = false;
            ControlBIBangEnable = true;
            //ROS_INFO_STREAM("++++++ BANG ENABLE...");
        }
    }



    if(ControlBIPidEnable)
    {
        bool sideReached = false;

        //********** SIDE

        //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
        //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        newError = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

        //double error;
        error = Side.Pid.Error;

        if(abs(Side.Pid.Error) < ObjLocMsg.SideThr)
        {
            Side.Pid.Output = 0;
            Side.Pid.Integral = 0;
            Side.Pid.LastError = error;

            sideReached = true;
            //ROS_INFO_STREAM("______ SIDE REACHED...");
        }
        else {
    //            error = sqrt(abs(Side.Pid.Error) - ObjLocMsg.SideThr);
    //            if(Side.Pid.Error < 0)
    //                error = -error;

            // integral term
            Side.Pid.Integral += error * Side.Pid.DeltaT;
            // clamp windup
            if(Side.Pid.Integral < -Side.Pid.IntegralWindupGuard)
                Side.Pid.Integral = -Side.Pid.IntegralWindupGuard;
            else if(Side.Pid.Integral > Side.Pid.IntegralWindupGuard)
                Side.Pid.Integral = Side.Pid.IntegralWindupGuard;

            // derivative term
            double deltaError = error - Side.Pid.LastError;
            Side.Pid.Derivative = deltaError / Side.Pid.DeltaT;
            Side.Pid.LastError = error;
    //        if(Side.Pid.FirstDerivative == true)
    //        {
    //            Side.Pid.Derivative = 0;
    //            Side.Pid.FirstDerivative = false;
    //        }


            Side.Pid.Output = (Side.Pid.Kp * error) +
                    (Side.Pid.Ki * Side.Pid.Integral) +
                    (Side.Pid.Kd * Side.Pid.Derivative);
        }

        //positive -> moves left
        MavCommand.linear.y = Side.Pid.Output;
        //ROS_INFO_STREAM("Pre " << ObjLocMsg.Xc << " Error " << newError << " U " << Side.Pid.Output);


    //    if(HoveringOffCounter == 0 && sideReached && frontReached && HoveringEnable)
    //    {
    //        // do hovering
    //        HoveringCounter = HoveringSteps;
    //        HoveringOffCounter = 2*HoveringSteps;

    //        MavCommand.angular.x = 0;
    //        MavCommand.angular.y = 0;
    //        MavCommand.angular.z = 0;
    //        MavCommand.linear.x = 0;
    //        MavCommand.linear.y = 0;
    //        //MavCommand.linear.z = 0;
    //    }
    //    else
//        if(sideReached && frontReached && altReached)
        if(sideReached)
        {
            //HoveringEnable = false;
            ControlBIBangEnable = true;
            ControlBIBangState = 0;
            ControlBIPidEnable = false;

            ROS_INFO_STREAM("Center Reached");
            ROS_INFO_STREAM("++++++ BANG ENABLE");
            //MsgStr = "CENTER REACHED";
        }
    }



    if(ControlBIBangEnable)
    {
        if(ObjGrasperMsg.Score < ObjGrasperMsg.ScoreThr)
        {
            ControlBIBangState = 3; // go to default to abort
            //MavCommand.linear.x = 0;
            //MavCommand.linear.y = 0;
        }

        switch(ControlBIBangState)
        {
        case 0: {
            ControlBIBangState = 1;
            ControlBIBangCounter = 3;

            //********** SIDE

            //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
            //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
            int errorPixel = ObjLocMsg.XcPixel - ObjGrasperMsg.XcPixel;
            //Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

            if(abs(errorPixel) < 20)
                ControlBIBangSideOut = 0;
            else if(errorPixel < 0)
                ControlBIBangSideOut = ControlBISpeed;
            else
                ControlBIBangSideOut = -ControlBISpeed;

            //positive -> moves left
            MavCommand.linear.y = ControlBIBangSideOut;

            //ROS_INFO_STREAM("Pre " << ObjLocMsg.Xc << " Error " << newError << " U " << Side.Pid.Output);
            ROS_INFO_STREAM("______ BANG STATE 0...");
        } break;
        case 1: {
            if(ControlBIBangCounter > 0)
            {
                ControlBIBangCounter--;
                ControlBIBangState = 1;
                MavCommand.linear.y = ControlBIBangSideOut;
                //MavCommand.linear.x = ControlBIBangFrontOut;
            }
            else
            {
                ControlBIBangState = 2;
                ControlBIBangCounter = 60;
                MavCommand.linear.y = 0;
                //MavCommand.linear.x = 0;
            }
        } break;
        case 2: {
            if(ControlBIBangCounter > 0)
            {
                ControlBIBangCounter--;
                ControlBIBangState = 2;
            }
            else
            {
                ControlBIBangState = 0;
            }

            MavCommand.linear.y = 0;
            //MavCommand.linear.x = 0;
        } break;
        default: {
            ControlBIBangState = 0;
            MavCommand.linear.y = 0;
            //MavCommand.linear.x = 0;
        } break;
        } // end switch(ControlBIBangState)
    }


    MavCommand.angular.x = 0;
    MavCommand.angular.y = 0;
    MavCommand.angular.z = 0;

    // Update velocities
    MavCommandPub.publish(MavCommand);
    //ROS_INFO_STREAM("Ux " << MavCommand.linear.x << " Uy " << MavCommand.linear.y << " Uz " << MavCommand.linear.z << " Uwz " << MavCommand.angular.z);
}


void ControlFrontal::ControlByIntersection(void) {
// We use Bang-Bang for X and Y axis. It tends to push the MAV out of buckets FOV.
// Version for BEBOP 2.
// Individual control for  altitude, side and front velocities using
// ObjectLocalization CNN for BUCKET detection and SDD for GRASPER detection.
// The NN outputs:
// Object's orientation (quaternion), Not used here¡
// Object's center (x,y) (pixels)
// Object's height, width (pixels)
// Object's confidence [0,1]
// Object's Scale Factor [-1,1], 0 -> 0.5m, Not used here¡

    if(ObjLocMsg.Score > ObjLocMsg.ScoreThr)
    {   // add object region and top view yaw orientation
        ObjLoc_DrawBucket(MavImg);
        // draw intersection threshold
        cv::circle(MavImg, cv::Point(ObjLocMsg.XcPixel, ObjLocMsg.YcPixel), 70, cv::Scalar(255,255,0), 1);
    }

    if(ObjGrasperMsg.Score > ObjGrasperMsg.ScoreThr)
    {
        ObjLoc_DrawGrasper(MavImg);
    }

    if(AutoControl == 0) {
        cv::putText(MavImg, "CONTROLLER OFF", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 0, 0), 2, 2);
        //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
        MovIndex = 0;
        MsgStr = "CONTROLLER ON";
        ControlBIEnable = true;
        ControlBIPidEnable = false;
        ControlBIBangEnable = true;
        HoveringEnable = true;
        LiftEnable = false;
        LiftState = 0;
        LiftCounter = LiftSteps;
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }

    // disable PID control in lifting stage
    if(ControlBIEnable == false)
    {
        cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }


    ostringstream strHeight;
    //strHeight << ObjLocMsg.Altitude;
    strHeight << Altitude.Current;

    cv::putText(MavImg, "Height: " + strHeight.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
    //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
    cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);

    // draw threshold lines
    // horizontal lines
    double imgHCenter = ObjLocMsg.ImgH/2;
    int tmp = int(imgHCenter - ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    tmp = int(imgHCenter + ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    
    tmp = int(imgHCenter - 140);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    tmp = int(imgHCenter + 140);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);

    // vertical lines
    double imgWCenter = ObjLocMsg.ImgW/2;
    tmp = int(imgWCenter - ObjLocMsg.SideThrPixel - 0);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    tmp = int(imgWCenter + ObjLocMsg.SideThrPixel - 0);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    
    tmp = int(imgWCenter - 140);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    tmp = int(imgWCenter + 140);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);

    // draw intersection threshold
    //cv::circle(MavImg, cv::Point(int(imgWCenter), int(imgHCenter)), 70, cv::Scalar(255,255,0), 1);

    imshow("DroneImage", MavImg);
    cv::waitKey(1);



    //********** ALTITUDE

    bool altReached = false;
    double deltaError;
    // Moves in meters, refs must be > 0
    //Altitude.Pid.Error = Altitude.References[MovIndex] - ObjLocMsg.Altitude;
    Altitude.Pid.Error = Altitude.References[MovIndex] - Altitude.Current;

    // integral term
    Altitude.Pid.Integral += Altitude.Pid.Error * Altitude.Pid.DeltaT;
    // clamp windup
    if(Altitude.Pid.Integral < -Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = -Altitude.Pid.IntegralWindupGuard;
    else if(Altitude.Pid.Integral > Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = Altitude.Pid.IntegralWindupGuard;

    // derivative term
    deltaError = Altitude.Pid.Error - Altitude.Pid.LastError;
    Altitude.Pid.Derivative = deltaError / Altitude.Pid.DeltaT;
    Altitude.Pid.LastError = Altitude.Pid.Error;

    //Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error);

    if(abs(Altitude.Pid.Error) < 5*ObjLocMsg.AltThr)
    {
        Altitude.Pid.Output = 0;
        Altitude.Pid.Integral = 0;
        Altitude.Pid.LastError = Altitude.Pid.Error;
        altReached = true;
    }
    else {
        Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error) +
                (Altitude.Pid.Ki * Altitude.Pid.Integral) +
                (Altitude.Pid.Kd * Altitude.Pid.Derivative);
    }

    if(Altitude.Pid.Output > 0.5) Altitude.Pid.Output = 0.5;
    if(Altitude.Pid.Output < -0.5) Altitude.Pid.Output = -0.5;

    // positive -> moves up
    MavCommand.linear.z = Altitude.Pid.Output;


    //********* METRICS

//    if(Metrics.AccuracyTotal > 0)
//    {
//        Metrics.AccuracyTotal--;

//        if(ObjLocMsg.Score < ObjLocMsg.ScoreThr)
//        {
//            Metrics.AccuracyFail++;
//        }
//    }
//    else if(Metrics.AccuracyTotal == 0)
//    {
//        ROS_INFO_STREAM("+++++++++++++++++++ ACCURACY, Fail: " << Metrics.AccuracyFail);
//        // deactivate printing
//        Metrics.AccuracyTotal = -1;
//    }


    //********** HOVERING

    if(ObjLocMsg.Score < ObjLocMsg.ScoreThr) {
        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;

        Side.Pid.Integral = 0;
        Side.Pid.FirstDerivative = true;
        Front.Pid.Integral = 0;
        Front.Pid.FirstDerivative = true;
        Yaw.Pid.Integral = 0;
        Yaw.Pid.FirstDerivative = true;
        
        ControlBIPidEnable = false;
        ControlBIBangEnable = false;
        ControlBIBangState = 0;

        ROS_INFO_STREAM("NO OBJECT, HOVERING");

        MavCommandPub.publish(MavCommand);
        return;
    }

    if(HoveringCounter > 0)
    {
        HoveringCounter--;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;

        //ROS_INFO_STREAM("We are in hovering regions. HoveringCounter: " << HoveringCounter);

        MavCommandPub.publish(MavCommand);
        return;
    }

    // we allow hovering only for a fixed amount of time
    if(HoveringOffCounter > 0)
    {
        HoveringOffCounter--;
        //ROS_INFO_STREAM("HOVERING OFF " << HoveringOffCounter);
    }



    //********** INTERSECTION, distance between bucket and grasper

    int distance;
    distance = int( sqrt( double((ObjGrasperMsg.XcPixel - ObjLocMsg.XcPixel)*(ObjGrasperMsg.XcPixel - ObjLocMsg.XcPixel)) + double((ObjGrasperMsg.YcPixel - ObjLocMsg.YcPixel)*(ObjGrasperMsg.YcPixel - ObjLocMsg.YcPixel)) ) );

    if(distance < 70)
    {
        if(ObjGrasperMsg.Score >= ObjGrasperMsg.ScoreThr)
            ControlBICounter++;
    }

    if(ControlBIMaxCounter > 0)
    {
        ControlBIMaxCounter--;
    }
    else if(ControlBIMaxCounter == 0)
    {
        ControlBIMaxCounter = 90;
        ControlBIAcc = double(ControlBICounter) / double(ControlBIMaxCounter);
        ROS_INFO_STREAM("Intersection Counter: " << ControlBICounter << " Intersection Accuracy: " << ControlBIAcc);
        ControlBICounter = 0;

        if(ControlBIAcc > 0.5)
        {
            LiftEnable = true;
            ControlBIEnable = false;
            ROS_INFO_STREAM("****** READY TO LIFT...");

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            //MavCommand.linear.z = 0;

            MavCommandPub.publish(MavCommand);
            return;
        }
    }



    //*********** Go to center or use bang bang

    MavCommand.linear.x = 0;
    MavCommand.linear.y = 0;
    //MavCommand.linear.z = 0;

    if(ControlBIPidEnable == false)
    {
        int imgHCenter = int(ObjLocMsg.ImgH/2);
        int imgWCenter = int(ObjLocMsg.ImgW/2);

        int xDistance = abs(imgWCenter - ObjLocMsg.XcPixel);
        int yDistance = abs(imgHCenter - ObjLocMsg.YcPixel);

        if(xDistance > 140 || yDistance > 140)
        {
            ControlBIPidEnable = true;
            ControlBIBangEnable = false;
            ControlBIBangState = 0;
            ROS_INFO_STREAM("###### PID ENABLE...");
        }
        else
        {
            ControlBIPidEnable = false;
            ControlBIBangEnable = true;
            //ROS_INFO_STREAM("++++++ BANG ENABLE...");
        }
    }



    if(ControlBIPidEnable)
    {
        bool sideReached = false;
        bool frontReached = false;

        //********** SIDE

        //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
        //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        double newError = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

        double error;
        error = Side.Pid.Error;

        if(abs(Side.Pid.Error) < ObjLocMsg.SideThr)
        {
            Side.Pid.Output = 0;
            Side.Pid.Integral = 0;
            Side.Pid.LastError = error;

            sideReached = true;
            //ROS_INFO_STREAM("______ SIDE REACHED...");
        }
        else {
    //            error = sqrt(abs(Side.Pid.Error) - ObjLocMsg.SideThr);
    //            if(Side.Pid.Error < 0)
    //                error = -error;

            // integral term
            Side.Pid.Integral += error * Side.Pid.DeltaT;
            // clamp windup
            if(Side.Pid.Integral < -Side.Pid.IntegralWindupGuard)
                Side.Pid.Integral = -Side.Pid.IntegralWindupGuard;
            else if(Side.Pid.Integral > Side.Pid.IntegralWindupGuard)
                Side.Pid.Integral = Side.Pid.IntegralWindupGuard;

            // derivative term
            double deltaError = error - Side.Pid.LastError;
            Side.Pid.Derivative = deltaError / Side.Pid.DeltaT;
            Side.Pid.LastError = error;
    //        if(Side.Pid.FirstDerivative == true)
    //        {
    //            Side.Pid.Derivative = 0;
    //            Side.Pid.FirstDerivative = false;
    //        }


            Side.Pid.Output = (Side.Pid.Kp * error) +
                    (Side.Pid.Ki * Side.Pid.Integral) +
                    (Side.Pid.Kd * Side.Pid.Derivative);
        }

        //positive -> moves left
        MavCommand.linear.y = Side.Pid.Output;
        //ROS_INFO_STREAM("Pre " << ObjLocMsg.Xc << " Error " << newError << " U " << Side.Pid.Output);


        //********** FRONT

        //Front.Pid.Error = Front.References[MovIndex] - ObjLocMsg.YcPixel;
        //Front.Pid.Error = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
        newError = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
        Utils_MeanFilter(newError, Front.Pid.Error, PoseNetFilterSize);

        error = Front.Pid.Error;

        if(abs(Front.Pid.Error) < ObjLocMsg.FrontThr)
        {
            Front.Pid.Output = 0;
            Front.Pid.Integral = 0;
            Front.Pid.LastError = error;

            frontReached = true;
            //ROS_INFO_STREAM("______ FRONT REACHED...");
        }
        else {
    //            error = sqrt(abs(Front.Pid.Error) - ObjLocMsg.FrontThr);
    //            if(Front.Pid.Error < 0)
    //                error = -error;

            // integral term
            Front.Pid.Integral += error * Front.Pid.DeltaT;
            // clamp windup
            if(Front.Pid.Integral < -Front.Pid.IntegralWindupGuard)
                Front.Pid.Integral = -Front.Pid.IntegralWindupGuard;
            else if(Front.Pid.Integral > Front.Pid.IntegralWindupGuard)
                Front.Pid.Integral = Front.Pid.IntegralWindupGuard;

            // derivative term
            deltaError = error - Front.Pid.LastError;
            Front.Pid.Derivative = deltaError / Front.Pid.DeltaT;
            Front.Pid.LastError = error;
    //        if(Front.Pid.FirstDerivative == true)
    //        {
    //            Front.Pid.Derivative = 0;
    //            Front.Pid.FirstDerivative = false;
    //        }


            Front.Pid.Output = (Front.Pid.Kp * error) +
                    (Front.Pid.Ki * Front.Pid.Integral) +
                    (Front.Pid.Kd * Front.Pid.Derivative);
        }

        //positive -> moves front
        MavCommand.linear.x = Front.Pid.Output;
        //ROS_INFO_STREAM("Pre " << ObjLocMsg.Yc << " Error " << newError << " U " << Front.Pid.Output);



    //    if(HoveringOffCounter == 0 && sideReached && frontReached && HoveringEnable)
    //    {
    //        // do hovering
    //        HoveringCounter = HoveringSteps;
    //        HoveringOffCounter = 2*HoveringSteps;

    //        MavCommand.angular.x = 0;
    //        MavCommand.angular.y = 0;
    //        MavCommand.angular.z = 0;
    //        MavCommand.linear.x = 0;
    //        MavCommand.linear.y = 0;
    //        //MavCommand.linear.z = 0;
    //    }
    //    else
//        if(sideReached && frontReached && altReached)
        if(sideReached && frontReached)
        {
            //HoveringEnable = false;
            ControlBIBangEnable = true;
            ControlBIBangState = 0;
            ControlBIPidEnable = false;

            ROS_INFO_STREAM("Center Reached");
            ROS_INFO_STREAM("++++++ BANG ENABLE");
            //MsgStr = "CENTER REACHED";
        }
    }



    if(ControlBIBangEnable)
    {
        if(ObjGrasperMsg.Score < ObjGrasperMsg.ScoreThr)
        {
            ControlBIBangState = 3; // go to default to abort
            //MavCommand.linear.x = 0;
            //MavCommand.linear.y = 0;
        }
        
        switch(ControlBIBangState)
        {
        case 0: {
            ControlBIBangState = 1;
            ControlBIBangCounter = 3;

            //********** SIDE

            //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
            //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
            int error = ObjLocMsg.XcPixel - ObjGrasperMsg.XcPixel;
            //Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

            if(abs(error) < 20)
                ControlBIBangSideOut = 0;
            else if(error < 0)
                ControlBIBangSideOut = ControlBISpeed;
            else
                ControlBIBangSideOut = -ControlBISpeed;
            
            //positive -> moves left
            MavCommand.linear.y = ControlBIBangSideOut;

            //ROS_INFO_STREAM("Pre " << ObjLocMsg.Xc << " Error " << newError << " U " << Side.Pid.Output);


            //********** FRONT

            //Front.Pid.Error = Front.References[MovIndex] - ObjLocMsg.YcPixel;
            //Front.Pid.Error = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
            error = ObjLocMsg.YcPixel - ObjGrasperMsg.YcPixel;
            //Utils_MeanFilter(newError, Front.Pid.Error, PoseNetFilterSize);


            if(abs(error) < 20)
                ControlBIBangFrontOut = 0;
            else if(error < 0)
                ControlBIBangFrontOut = ControlBISpeed;
            else
                ControlBIBangFrontOut = -ControlBISpeed;
            
            //positive -> moves front
            MavCommand.linear.x = ControlBIBangFrontOut;

            ROS_INFO_STREAM("______ BANG STATE 0...");
        } break;
        case 1: {
            if(ControlBIBangCounter > 0)
            {
                ControlBIBangCounter--;
                ControlBIBangState = 1;
                MavCommand.linear.y = ControlBIBangSideOut;
                MavCommand.linear.x = ControlBIBangFrontOut;
            }
            else
            {
                ControlBIBangState = 2;
                ControlBIBangCounter = 60;
                MavCommand.linear.y = 0;
                MavCommand.linear.x = 0;
            }
        } break;
        case 2: {
            if(ControlBIBangCounter > 0)
            {
                ControlBIBangCounter--;
                ControlBIBangState = 2;
            }
            else
            {
                ControlBIBangState = 0;
            }
            
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
        } break;
        default: {
            ControlBIBangState = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
        } break;
        } // end switch(ControlBIBangState)
    }


    MavCommand.angular.x = 0;
    MavCommand.angular.y = 0;
    MavCommand.angular.z = 0;

    // Update velocities
    MavCommandPub.publish(MavCommand);
    //ROS_INFO_STREAM("Ux " << MavCommand.linear.x << " Uy " << MavCommand.linear.y << " Uz " << MavCommand.linear.z << " Uwz " << MavCommand.angular.z);
}


void ControlFrontal::ControlLift2(void)
{
    // Control lifting secuence when both BUCKET and GRASPER are detectec.
    // Is better if the grasper is attached at the front of the Bebop2.

    if(LiftEnable == false)
        return;

    switch(LiftState)
    {
    case 0: {
        // stay in this state when all is done

        if(LiftCounter > 0)
        {
            ROS_INFO_STREAM("****** LIFTING UP...");

            LiftCounter--;

            MsgStr = "++ LIFTING ++";

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            MavCommand.linear.z = 0.25;
        }
        else
        {
            MsgStr = "** ALL DONE **";

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            MavCommand.linear.z = 0;
        }

        MavCommandPub.publish(MavCommand);
    } break;

    default: {
        LiftState = 0;
    } break;
    } // end switch(LiftState)

}


void ControlFrontal::ControlLift(void)
{
    // This is a 'blind' Forward-Up control lifting secuence.
    // There is not detection of the grasper.
    // Is better if the grasper is attached at the back of the Bebop2.
    // The air will push back the grasper so most of the time the grasper will
    // be out of sight.
    // If the grasper is attached at the front of the Bebop2, the grasper will be
    // visible most of the time, but the Forward-Up lifting secuence wont work.

    if(LiftEnable == false)
        return;

    switch(LiftState)
    {
    case 0: {
        // ensure stable start point
        if(LiftCounter > 0)
        {
            LiftCounter--;
        }
        else {
            if(LiftPositiveCounter > LiftSteps)
                LiftPositiveCounter = LiftSteps;

            double acc = double(LiftPositiveCounter) / double(LiftSteps);
            if(acc > 0.5)
            {
                MsgStr = "READY TO LIFT...";
                ROS_INFO_STREAM("****** READY TO LIFT...");
                LiftState = 1;

                // disable PID control, set counter, set velocities
                ControlSEnable = false;
                LiftCounter = 90;
                MavCommand.angular.x = 0;
                MavCommand.angular.y = 0;
                MavCommand.angular.z = 0;
                MavCommand.linear.x = 0;
                MavCommand.linear.y = 0;
                MavCommand.linear.z = 0;
                MavCommandPub.publish(MavCommand);
            }
            else {
                LiftEnable = false;
                LiftPositiveCounter = 0;
                LiftCounter = LiftSteps;
            }

            ROS_INFO_STREAM("****** LIFT Accuracy: " << acc << " LiftPositiveCounter " << LiftPositiveCounter);
        }
    } break;

    case 1: {
        ROS_INFO_STREAM("****** LIFTING FORWARD...");

        if(LiftCounter > 0)
        {
            LiftCounter--;

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0.035;
            MavCommand.linear.y = -0.013;
            MavCommand.linear.z = 0;
        }
        else
        {
            LiftState = 2;
            LiftCounter = 15;

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            MavCommand.linear.z = 0;
        }

        MavCommandPub.publish(MavCommand);
    } break;

    case 2: {
        ROS_INFO_STREAM("****** LIFTING HOVERING...");

        if(LiftCounter > 0)
        {
            LiftCounter--;

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            MavCommand.linear.z = 0;
        }
        else
        {
            LiftState = 3;
            LiftCounter = 60;

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            MavCommand.linear.z = 0;
        }

        MavCommandPub.publish(MavCommand);
    } break;

    case 3: {
        // stay in this state when all done

        if(LiftCounter > 0)
        {
            ROS_INFO_STREAM("****** LIFTING UP...");

            LiftCounter--;

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            MavCommand.linear.z = 0.25;
        }
        else
        {
            MsgStr = "** ALL DONE **";

            MavCommand.angular.x = 0;
            MavCommand.angular.y = 0;
            MavCommand.angular.z = 0;
            MavCommand.linear.x = 0;
            MavCommand.linear.y = 0;
            MavCommand.linear.z = 0;
        }

        MavCommandPub.publish(MavCommand);
    } break;

    default: {
        LiftState = 0;
    } break;
    } // end switch(LiftState)

}


void ControlFrontal::PID_Control6B(void) {
// Version for BEBOP 2.
// Individual control for yaw, altitude, side and front velocities using
// ObjectLocalization Neural Network and pixel references.
// The NN outputs:
// Object's orientation (quaternion)
// Object's center (x,y) (pixels)
// Object's height, width (pixels)
// Object's confidence [0,1]
// Object's Scale Factor [-1,1], 0 -> 0.5m

    if(ObjLocMsg.Score > ObjLocMsg.ScoreThr)
    {   // add object region and top view yaw orientation
        ObjLoc_DrawBucketWithYaw(MavImg);
        ostringstream strYaw;
        strYaw << ObjLocMsg.Yaw * 180/M_PI;
        cv::putText(MavImg, " Yaw: " + strYaw.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
    }

    if(AutoControl == 0) {
        cv::putText(MavImg, "CONTROLLER OFF", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 0, 0), 2, 2);
        //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
        MovIndex = 0;
        MsgStr = "CONTROLLER ON";
        ControlSEnable = true;
        HoveringEnable = true;
        LiftEnable = false;
        LiftState = 0;
        YawReached = false;
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }

    // disable PID control in lifting stage
    if(ControlSEnable == false)
    {
        cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }


    ostringstream strYaw, strHeight;
    strHeight << ObjLocMsg.Altitude;
    strYaw << ObjLocMsg.Yaw * 180/M_PI;

    cv::putText(MavImg, " Yaw: " + strYaw.str() + " " + " Height: " + strHeight.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
    //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
    cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);

    // draw threshold lines
    // horizontal lines
    int tmp = int(ObjLocMsg.ImgH/2 - ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    tmp = int(ObjLocMsg.ImgH/2 + ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    // vertical lines
    tmp = int(ObjLocMsg.ImgW/2 - ObjLocMsg.SideThrPixel - 0);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    tmp = int(ObjLocMsg.ImgW/2 + ObjLocMsg.SideThrPixel - 0);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);

    imshow("DroneImage", MavImg);
    cv::waitKey(1);


    //********** ALTITUDE

    bool altReached = false;
    double deltaError;
    // Moves in meters, refs must be > 0
    //Altitude.Pid.Error = Altitude.References[MovIndex] - ObjLocMsg.Altitude;
    Altitude.Pid.Error = Altitude.References[MovIndex] - Altitude.Current;

    // integral term
    Altitude.Pid.Integral += Altitude.Pid.Error * Altitude.Pid.DeltaT;
    // clamp windup
    if(Altitude.Pid.Integral < -Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = -Altitude.Pid.IntegralWindupGuard;
    else if(Altitude.Pid.Integral > Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = Altitude.Pid.IntegralWindupGuard;

    // derivative term
    deltaError = Altitude.Pid.Error - Altitude.Pid.LastError;
    Altitude.Pid.Derivative = deltaError / Altitude.Pid.DeltaT;
    Altitude.Pid.LastError = Altitude.Pid.Error;

    //Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error);

    if(abs(Altitude.Pid.Error) < 5*ObjLocMsg.AltThr)
    {
        Altitude.Pid.Output = 0;
        Altitude.Pid.Integral = 0;
        Altitude.Pid.LastError = Altitude.Pid.Error;
        altReached = true;
    }
    else {
        Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error) +
                (Altitude.Pid.Ki * Altitude.Pid.Integral) +
                (Altitude.Pid.Kd * Altitude.Pid.Derivative);
    }

    if(Altitude.Pid.Output > 0.5) Altitude.Pid.Output = 0.5;
    if(Altitude.Pid.Output < -0.5) Altitude.Pid.Output = -0.5;

    // positive -> moves up
    MavCommand.linear.z = Altitude.Pid.Output;


    //********* METRICS

    if(Metrics.AccuracyTotal > 0)
    {
        Metrics.AccuracyTotal--;

        if(ObjLocMsg.Score < ObjLocMsg.ScoreThr)
        {
            Metrics.AccuracyFail++;
        }
    }
    else if(Metrics.AccuracyTotal == 0)
    {
        ROS_INFO_STREAM("+++++++++++++++++++ ACCURACY, Fail: " << Metrics.AccuracyFail);
        // deactivate printing
        Metrics.AccuracyTotal = -1;
    }


    //********** HOVERING

    if(ObjLocMsg.Score < ObjLocMsg.ScoreThr) {
        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;

        Side.Pid.Integral = 0;
        Side.Pid.FirstDerivative = true;
        Front.Pid.Integral = 0;
        Front.Pid.FirstDerivative = true;
        Yaw.Pid.Integral = 0;
        Yaw.Pid.FirstDerivative = true;

        ROS_INFO_STREAM("NO OBJECT, HOVERING");

        MavCommandPub.publish(MavCommand);
        return;
    }

    if(HoveringCounter > 0)
    {
        HoveringCounter--;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;

        //ROS_INFO_STREAM("We are in hovering regions. HoveringCounter: " << HoveringCounter);

        MavCommandPub.publish(MavCommand);
        return;
    }

    // we allow hovering only for a fixed amount of time
    if(HoveringOffCounter > 0)
    {
        HoveringOffCounter--;
        //ROS_INFO_STREAM("HOVERING OFF " << HoveringOffCounter);
    }


    bool sideReached = false;
    bool frontReached = false;
    //bool yawReached = false;



    //********** SIDE

    //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
    //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
    double newError = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
    Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

    double error = Side.Pid.Error;
    error = Side.Pid.Error;

    if(abs(Side.Pid.Error) < ObjLocMsg.SideThr)
    {
        Side.Pid.Output = 0;
        Side.Pid.Integral = 0;
        Side.Pid.LastError = error;

        sideReached = true;
    }
    else {
//            error = sqrt(abs(Side.Pid.Error) - ObjLocMsg.SideThr);
//            if(Side.Pid.Error < 0)
//                error = -error;

        // integral term
        Side.Pid.Integral += error * Side.Pid.DeltaT;
        // clamp windup
        if(Side.Pid.Integral < -Side.Pid.IntegralWindupGuard)
            Side.Pid.Integral = -Side.Pid.IntegralWindupGuard;
        else if(Side.Pid.Integral > Side.Pid.IntegralWindupGuard)
            Side.Pid.Integral = Side.Pid.IntegralWindupGuard;

        // derivative term
        double deltaError = error - Side.Pid.LastError;
        Side.Pid.Derivative = deltaError / Side.Pid.DeltaT;
        Side.Pid.LastError = error;
//        if(Side.Pid.FirstDerivative == true)
//        {
//            Side.Pid.Derivative = 0;
//            Side.Pid.FirstDerivative = false;
//        }


        Side.Pid.Output = (Side.Pid.Kp * error) +
                (Side.Pid.Ki * Side.Pid.Integral) +
                (Side.Pid.Kd * Side.Pid.Derivative);
    }

    //positive -> moves left
    MavCommand.linear.y = Side.Pid.Output;
    //ROS_INFO_STREAM("Pre " << ObjLocMsg.Xc << " Error " << newError << " U " << Side.Pid.Output);


    //********** FRONT

    //Front.Pid.Error = Front.References[MovIndex] - ObjLocMsg.YcPixel;
    //Front.Pid.Error = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
    newError = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
    Utils_MeanFilter(newError, Front.Pid.Error, PoseNetFilterSize);

    error = Front.Pid.Error;

    if(abs(Front.Pid.Error) < ObjLocMsg.FrontThr)
    {
        Front.Pid.Output = 0;
        Front.Pid.Integral = 0;
        Front.Pid.LastError = error;

        frontReached = true;
    }
    else {
//            error = sqrt(abs(Front.Pid.Error) - ObjLocMsg.FrontThr);
//            if(Front.Pid.Error < 0)
//                error = -error;

        // integral term
        Front.Pid.Integral += error * Front.Pid.DeltaT;
        // clamp windup
        if(Front.Pid.Integral < -Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = -Front.Pid.IntegralWindupGuard;
        else if(Front.Pid.Integral > Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = Front.Pid.IntegralWindupGuard;

        // derivative term
        deltaError = error - Front.Pid.LastError;
        Front.Pid.Derivative = deltaError / Front.Pid.DeltaT;
        Front.Pid.LastError = error;
//        if(Front.Pid.FirstDerivative == true)
//        {
//            Front.Pid.Derivative = 0;
//            Front.Pid.FirstDerivative = false;
//        }


        Front.Pid.Output = (Front.Pid.Kp * error) +
                (Front.Pid.Ki * Front.Pid.Integral) +
                (Front.Pid.Kd * Front.Pid.Derivative);
    }

    //positive -> moves front
    MavCommand.linear.x = Front.Pid.Output;
    //ROS_INFO_STREAM("Pre " << ObjLocMsg.Yc << " Error " << newError << " U " << Front.Pid.Output);



    if(HoveringOffCounter == 0 && sideReached && frontReached && HoveringEnable)
    {
        // do hovering
        HoveringCounter = HoveringSteps;
        HoveringOffCounter = 2*HoveringSteps;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;
    }
    else
    if(sideReached && frontReached)
    {
        //PoseNetFilterSize = 10;

        //********** YAW

        // dont turn in fine tune area
        if(YawReached == false)
        {
            // Referencia en grados
    //#ifdef BEBOP2
            //Yaw.Pid.Error = Yaw.References[MovIndex] - ObjLocMsg.Yaw*180/M_PI;
    //#else
            Yaw.Pid.Error = ObjLocMsg.Yaw*180/M_PI - Yaw.References[MovIndex]; // tum_simulator
    //#endif

            // integral term
            Yaw.Pid.Integral += Yaw.Pid.Error * Yaw.Pid.DeltaT;
            // clamp windup
            if(Yaw.Pid.Integral < -Yaw.Pid.IntegralWindupGuard)
                Yaw.Pid.Integral = -Yaw.Pid.IntegralWindupGuard;
            else if(Yaw.Pid.Integral > Yaw.Pid.IntegralWindupGuard)
                Yaw.Pid.Integral = Yaw.Pid.IntegralWindupGuard;

            // derivative term
            deltaError = Yaw.Pid.Error - Yaw.Pid.LastError;
            Yaw.Pid.Derivative = deltaError / Yaw.Pid.DeltaT;
            Yaw.Pid.LastError = Yaw.Pid.Error;


            if(abs(Yaw.Pid.Error) < ObjLocMsg.YawThr)
            {
                Yaw.Pid.Output = 0;
                Yaw.Pid.Integral = 0;
                Yaw.Pid.LastError = Yaw.Pid.Error;

                YawReached = true;
            }
            else {
                HoveringEnable = true;

                Yaw.Pid.Output = (Yaw.Pid.Kp * Yaw.Pid.Error) +
                        (Yaw.Pid.Ki * Yaw.Pid.Integral) +
                        (Yaw.Pid.Kd * Yaw.Pid.Derivative);
            }

            // positive -> turns right?
            MavCommand.angular.z = Yaw.Pid.Output;
        }



        if(YawReached && altReached)
        {
            HoveringEnable = false;
            LiftEnable = true;
            LiftPositiveCounter++;

            PoseNetFilterSize = 1;
//            Side.Pid.Kd = 0.15;
//            Front.Pid.Kd = 0.15;
            ObjLocMsg.SideThrPixel = 15;
            ObjLocMsg.FrontThrPixel = 15;
            ObjLocMsg.SideThr = ObjLocMsg.SideThrPixel / ObjLocMsg.ImgW;
            ObjLocMsg.FrontThr = ObjLocMsg.FrontThrPixel / ObjLocMsg.ImgH;


            ROS_INFO_STREAM("REFERENCE REACHED");
            MsgStr = "REFERENCE REACHED";

            ROS_INFO_STREAM("Pose Bebop");
            ROS_INFO_STREAM(ViconBebopPose.transform.translation.x
                            << " " << ViconBebopPose.transform.translation.y
                            << " " << ViconBebopPose.transform.translation.z
                            << " " << ViconBebopPose.transform.rotation.w
                            << " " << ViconBebopPose.transform.rotation.x
                            << " " << ViconBebopPose.transform.rotation.y
                            << " " << ViconBebopPose.transform.rotation.z );
//                ROS_INFO_STREAM("Pose Bote");
//                ROS_INFO_STREAM(ViconBotePose.transform.translation.x
//                                << " " << ViconBotePose.transform.translation.y
//                                << " " << ViconBotePose.transform.translation.z
//                                << " " << ViconBotePose.transform.rotation.w
//                                << " " << ViconBotePose.transform.rotation.x
//                                << " " << ViconBotePose.transform.rotation.y
//                                << " " << ViconBotePose.transform.rotation.z );
            ROS_INFO_STREAM("ObjLoc");
            ROS_INFO_STREAM(ObjLocMsg.Altitude
                            << " " << ObjLocMsg.Yaw
                            << " " << ObjLocMsg.SF
                            << " " << ObjLocMsg.XcPixel
                            << " " << ObjLocMsg.YcPixel
                            << " " << ObjLocMsg.Q.w()
                            << " " << ObjLocMsg.Q.x()
                            << " " << ObjLocMsg.Q.y()
                            << " " << ObjLocMsg.Q.z() );
        }

    }
    else {
        MavCommand.angular.z = 0;
        MsgStr = "CONTROLLER ON";
    }

    MavCommand.angular.x = 0;
    MavCommand.angular.y = 0;

    // Update velocities
    MavCommandPub.publish(MavCommand);
    //ROS_INFO_STREAM("Ux " << MavCommand.linear.x << " Uy " << MavCommand.linear.y << " Uz " << MavCommand.linear.z << " Uwz " << MavCommand.angular.z);
}


void ControlFrontal::PID_Control6A(void) {
// Last stable version for BEBOP 1.
// Individual control for yaw, altitude, side and front velocities using
// ObjectLocalization Neural Network and pixel references.
// The NN outputs:
// Object's orientation (quaternion)
// Object's center (x,y) (pixels)
// Object's height, width (pixels)
// Object's confidence [0,1]
// Object's Scale Factor [-1,1], 0 -> 0.5m

    if(ObjLocMsg.Score > ObjLocMsg.ScoreThr)
        // add region, line of yaw
        ObjLoc_DrawBucketWithYaw(MavImg);

    if(AutoControl == 0) {
        cv::putText(MavImg, " CONTROLLER OFF ", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 0, 0), 2, 2);
        //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
        MovState = MOV4_SIDE;
        MovIndex = 0;
        MsgStr = " CONTROLLER ON";
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }

    ostringstream strYaw, strHeight;
    strHeight << ObjLocMsg.Altitude;
    strYaw << ObjLocMsg.Yaw * 180/M_PI;

    cv::putText(MavImg, " Yaw: " + strYaw.str() + " " + " Height: " + strHeight.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
    //cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
    cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);

    // draw threshold lines
    // horizontal lines
    int tmp = int(ObjLocMsg.ImgH/2 - ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    tmp = int(ObjLocMsg.ImgH/2 + ObjLocMsg.FrontThrPixel);
    cv::line(MavImg, cv::Point(0, tmp), cv::Point(int(ObjLocMsg.ImgW), tmp), cv::Scalar(255,255,0), 1);
    // vertical lines
    tmp = int(ObjLocMsg.ImgW/2 - ObjLocMsg.SideThrPixel);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);
    tmp = int(ObjLocMsg.ImgW/2 + ObjLocMsg.SideThrPixel);
    cv::line(MavImg, cv::Point(tmp, 0), cv::Point(tmp, int(ObjLocMsg.ImgH)), cv::Scalar(255,255,0), 1);

    imshow("DroneImage", MavImg);
    cv::waitKey(1);


    //********** ALTITUDE

    bool altReached = false;
    double deltaError;
    // Moves in meters, refs must be > 0
    //Altitude.Pid.Error = Altitude.References[MovIndex] - ObjLocMsg.Altitude;
    Altitude.Pid.Error = Altitude.References[MovIndex] - Altitude.Current;

    // integral term
    Altitude.Pid.Integral += Altitude.Pid.Error * Altitude.Pid.DeltaT;
    // clamp windup
    if(Altitude.Pid.Integral < -Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = -Altitude.Pid.IntegralWindupGuard;
    else if(Altitude.Pid.Integral > Altitude.Pid.IntegralWindupGuard)
        Altitude.Pid.Integral = Altitude.Pid.IntegralWindupGuard;

    // derivative term
    deltaError = Altitude.Pid.Error - Altitude.Pid.LastError;
    Altitude.Pid.Derivative = deltaError / Altitude.Pid.DeltaT;
    Altitude.Pid.LastError = Altitude.Pid.Error;

    //Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error);

    if(abs(Altitude.Pid.Error) < 5*ObjLocMsg.AltThr)
    {
        Altitude.Pid.Output = 0;
        Altitude.Pid.Integral = 0;
        altReached = true;
    }
    else {
        Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error) +
                (Altitude.Pid.Ki * Altitude.Pid.Integral) +
                (Altitude.Pid.Kd * Altitude.Pid.Derivative);
    }

    if(Altitude.Pid.Output > 0.5) Altitude.Pid.Output = 0.5;
    if(Altitude.Pid.Output < -0.5) Altitude.Pid.Output = -0.5;

    // positive -> moves up
    MavCommand.linear.z = Altitude.Pid.Output;



    //********** HOVERING

    if(ObjLocMsg.Score < ObjLocMsg.ScoreThr) {
        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;
        MavCommandPub.publish(MavCommand);
        std::cout << "NO OBJECT, HOVERING" << std::endl;
        return;
    }

    if(HoveringCounter > 0)
    {
        HoveringCounter--;
        std::cout << "We are in hovering regions: HoveringCounter: " << HoveringCounter << std::endl;
    }
    else {
        MovState = MOV4_SIDE;
    }

    bool sideReached = false;
    bool frontReached = false;
    bool yawReached = false;

    switch(MovState) {
    case MOV4_SIDE: {
        //********** SIDE

        //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
        //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        double newError = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

        double error = Side.Pid.Error;

        if(abs(Side.Pid.Error) < ObjLocMsg.SideThr)
        {
            Side.Pid.Output = 0;
            Side.Pid.Integral = 0;
            sideReached = true;
        }
        else {
            error = sqrt(abs(Side.Pid.Error) - ObjLocMsg.SideThr);
            if(Side.Pid.Error < 0)
                error = -error;

            // integral term
            Side.Pid.Integral += error * Side.Pid.DeltaT;
            // clamp windup
            if(Side.Pid.Integral < -Side.Pid.IntegralWindupGuard)
                Side.Pid.Integral = -Side.Pid.IntegralWindupGuard;
            else if(Side.Pid.Integral > Side.Pid.IntegralWindupGuard)
                Side.Pid.Integral = Side.Pid.IntegralWindupGuard;

            // derivative term
            double deltaError = error - Side.Pid.LastError;
            Side.Pid.Derivative = deltaError / Side.Pid.DeltaT;
            Side.Pid.LastError = error;


            Side.Pid.Output = (Side.Pid.Kp * error) +
                    (Side.Pid.Ki * Side.Pid.Integral) +
                    (Side.Pid.Kd * Side.Pid.Derivative);
        }

        //positive -> moves left
        MavCommand.linear.y = Side.Pid.Output;
        //ROS_INFO_STREAM("Pre " << ObjLocMsg.Xc << " Error " << newError << " U " << Side.Pid.Output);


        //********** FRONT

        //Front.Pid.Error = Front.References[MovIndex] - ObjLocMsg.YcPixel;
        //Front.Pid.Error = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
        newError = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
        Utils_MeanFilter(newError, Front.Pid.Error, PoseNetFilterSize);

        error = Front.Pid.Error;

        if(abs(Front.Pid.Error) < ObjLocMsg.FrontThr)
        {
            Front.Pid.Output = 0;
            Front.Pid.Integral = 0;
            frontReached = true;
        }
        else {
            error = sqrt(abs(Front.Pid.Error) - ObjLocMsg.FrontThr);
            if(Front.Pid.Error < 0)
                error = -error;

            // integral term
            Front.Pid.Integral += error * Front.Pid.DeltaT;
            // clamp windup
            if(Front.Pid.Integral < -Front.Pid.IntegralWindupGuard)
                Front.Pid.Integral = -Front.Pid.IntegralWindupGuard;
            else if(Front.Pid.Integral > Front.Pid.IntegralWindupGuard)
                Front.Pid.Integral = Front.Pid.IntegralWindupGuard;

            // derivative term
            deltaError = error - Front.Pid.LastError;
            Front.Pid.Derivative = deltaError / Front.Pid.DeltaT;
            Front.Pid.LastError = error;


            Front.Pid.Output = (Front.Pid.Kp * error) +
                    (Front.Pid.Ki * Front.Pid.Integral) +
                    (Front.Pid.Kd * Front.Pid.Derivative);
        }

        //positive -> moves front
        MavCommand.linear.x = Front.Pid.Output;
        //ROS_INFO_STREAM("Pre " << ObjLocMsg.Yc << " Error " << newError << " U " << Front.Pid.Output);



        if(sideReached && frontReached)
        {
            PoseNetFilterSize = 10;

            //********** YAW

            // Referencia en grados
//#ifdef BEBOP2
            //Yaw.Pid.Error = Yaw.References[MovIndex] - ObjLocMsg.Yaw*180/M_PI;
//#else
            Yaw.Pid.Error = ObjLocMsg.Yaw*180/M_PI - Yaw.References[MovIndex]; // tum_simulator
//#endif

            // integral term
            Yaw.Pid.Integral += Yaw.Pid.Error * Yaw.Pid.DeltaT;
            // clamp windup
            if(Yaw.Pid.Integral < -Yaw.Pid.IntegralWindupGuard)
                Yaw.Pid.Integral = -Yaw.Pid.IntegralWindupGuard;
            else if(Yaw.Pid.Integral > Yaw.Pid.IntegralWindupGuard)
                Yaw.Pid.Integral = Yaw.Pid.IntegralWindupGuard;

            // derivative term
            deltaError = Yaw.Pid.Error - Yaw.Pid.LastError;
            Yaw.Pid.Derivative = deltaError / Yaw.Pid.DeltaT;
            Yaw.Pid.LastError = Yaw.Pid.Error;


            if(abs(Yaw.Pid.Error) < ObjLocMsg.YawThr)
            {
                Yaw.Pid.Output = 0;
                Yaw.Pid.Integral = 0;
                yawReached = true;
            }
            else {
                Yaw.Pid.Output = (Yaw.Pid.Kp * Yaw.Pid.Error) +
                        (Yaw.Pid.Ki * Yaw.Pid.Integral) +
                        (Yaw.Pid.Kd * Yaw.Pid.Derivative);
            }

            // positive -> turns right?
            MavCommand.angular.z = Yaw.Pid.Output;



            if(yawReached && altReached)
            {
                // Stop Yaw speed
                MavCommand.angular.z = 0;
                MavCommand.linear.x = 0;
                MavCommand.linear.y = 0;
                //MavCommand.linear.z = 0;
                ROS_INFO_STREAM("REFERENCE REACHED");
                MsgStr = "REFERENCE REACHED";
                //MovState = MOV4_STANDBY;

                ROS_INFO_STREAM("Pose Bebop");
                ROS_INFO_STREAM(ViconBebopPose.transform.translation.x
                                << " " << ViconBebopPose.transform.translation.y
                                << " " << ViconBebopPose.transform.translation.z
                                << " " << ViconBebopPose.transform.rotation.w
                                << " " << ViconBebopPose.transform.rotation.x
                                << " " << ViconBebopPose.transform.rotation.y
                                << " " << ViconBebopPose.transform.rotation.z );
//                ROS_INFO_STREAM("Pose Bote");
//                ROS_INFO_STREAM(ViconBotePose.transform.translation.x
//                                << " " << ViconBotePose.transform.translation.y
//                                << " " << ViconBotePose.transform.translation.z
//                                << " " << ViconBotePose.transform.rotation.w
//                                << " " << ViconBotePose.transform.rotation.x
//                                << " " << ViconBotePose.transform.rotation.y
//                                << " " << ViconBotePose.transform.rotation.z );
                ROS_INFO_STREAM("ObjLoc");
                ROS_INFO_STREAM(ObjLocMsg.Altitude
                                << " " << ObjLocMsg.Yaw
                                << " " << ObjLocMsg.SF
                                << " " << ObjLocMsg.XcPixel
                                << " " << ObjLocMsg.YcPixel
                                << " " << ObjLocMsg.Q.w()
                                << " " << ObjLocMsg.Q.x()
                                << " " << ObjLocMsg.Q.y()
                                << " " << ObjLocMsg.Q.z() );
            }

        }
        else {
            MavCommand.angular.z = 0;
            MsgStr = "CONTROLLER ON";
        }

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        //ROS_INFO_STREAM("Ux " << MavCommand.linear.x << " Uy " << MavCommand.linear.y << " Uz " << MavCommand.linear.z << " Uwz " << MavCommand.angular.z);


        // we allow hovering only for a fixed amount of time
        if(HoveringOffCounter > 0) {
            HoveringOffCounter--;
            std::cout << "HOVERING OFF " << HoveringOffCounter << std::endl;
        }

    } break;

    case MOV4_STANDBY: {
        // Mantener ultimo estado
        //MovIndex = 0;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        //MavCommand.linear.z = 0;
    } break;

    default:
        break;
    } // end switch(MovState)

    // Update velocities
    MavCommandPub.publish(MavCommand);
    //ROS_INFO_STREAM("Ux " << MavCommand.linear.x << " Uy " << MavCommand.linear.y << " Uz " << MavCommand.linear.z << " Uwz " << MavCommand.angular.z);
}


void ControlFrontal::PID_Control6(void) {
// This version was used for ICUAS metrics.
// Individual control for yaw, altitude, side and front velocities using
// ObjectLocalization Neural Network and pixel references.
// The NN outputs:
// Object's orientation (quaternion)
// Object's center (x,y) (pixels)
// Object's height, width (pixels)
// Object's confidence [0,1]
// Object's Scale Factor [-1,1], 0 -> 0.5m

    if(ObjLocMsg.Score > ObjLocMsg.ScoreThr)
        // add region, line of yaw
        ObjLoc_DrawBucketWithYaw(MavImg);

    if(AutoControl == 0) {
        cv::putText(MavImg, " CONTROLLER OFF ", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 0, 0), 2, 2);
        cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
        MovState = MOV4_SIDE;
        MovIndex = 0;
        MsgStr = " CONTROLLER ON";
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }

    ostringstream strYaw, strHeight;
    strHeight << ObjLocMsg.Altitude;
    strYaw << ObjLocMsg.Yaw * 180/M_PI;

    cv::putText(MavImg, " Yaw: " + strYaw.str() + " " + " Height: " + strHeight.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
    cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
    cv::putText(MavImg, MsgStr, Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);
    imshow("DroneImage", MavImg);
    cv::waitKey(1);

    //MovState = MOV4_SIDE;
//    if(ObjLocMsg.Score < ObjLocMsg.ScoreThr) {
//        //MovState = MOV4_STANDBY;
//        MavCommand.angular.x = 0;
//        MavCommand.angular.y = 0;
//        MavCommand.angular.z = 0;
//        MavCommand.linear.x = 0;
//        MavCommand.linear.y = 0;
//        MavCommand.linear.z = 0;
//        MavCommandPub.publish(MavCommand);
//        return;
//    }

    switch(MovState) {
    case MOV4_SIDE: {
        //********** SIDE

        //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
        //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        double newError = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

        // integral term
        Side.Pid.Integral += Side.Pid.Error * Side.Pid.DeltaT;
        // clamp windup
        if(Side.Pid.Integral < -Side.Pid.IntegralWindupGuard)
            Side.Pid.Integral = -Side.Pid.IntegralWindupGuard;
        else if(Side.Pid.Integral > Side.Pid.IntegralWindupGuard)
            Side.Pid.Integral = Side.Pid.IntegralWindupGuard;

        // derivative term
        double deltaError = Side.Pid.Error - Side.Pid.LastError;
        Side.Pid.Derivative = deltaError / Side.Pid.DeltaT;
        Side.Pid.LastError = Side.Pid.Error;

        // three stage control
        if(abs(Side.Pid.Error) < 3*ObjLocMsg.SideThr)
        {
            Side.Pid.Output = 0;
            Side.Pid.Integral = 0;
        }
//        else if(abs(Side.Pid.Error) < 5*ObjLocMsg.SideThr)
//        {
//            Side.Pid.Output = (Side.Pid.Kp * Side.Pid.Error)*0.01;
//                //(Side.Pid.Ki * Side.Pid.Integral);

//            //Side.Pid.Output = Side.Pid.Output*0.01;
//            //Side.Pid.Integral = 0;
//        }
//        else if(abs(Side.Pid.Error) < 8*ObjLocMsg.SideThr)
//        {
//            Side.Pid.Output = (Side.Pid.Kp * Side.Pid.Error) +
//                    (Side.Pid.Ki * Side.Pid.Integral) +
//                    (Side.Pid.Kd * Side.Pid.Derivative);
//        }
        else {
            //Side.Pid.Output = (Side.Pid.Kp * Side.Pid.Error);
//            Side.Pid.Output = (Side.Pid.Kp * Side.Pid.Error);
//                    //(Side.Pid.Kd * Side.Pid.Derivative);
//            Side.Pid.Integral = 0;

            Side.Pid.Output = (Side.Pid.Kp * Side.Pid.Error) +
                    (Side.Pid.Ki * Side.Pid.Integral) +
                    (Side.Pid.Kd * Side.Pid.Derivative);
        }

//        if(Side.Pid.Output < -0.1)
//            Side.Pid.Output = -0.1;
//        else if(Side.Pid.Output > 0.1)
//            Side.Pid.Output = 0.1;

//        if(newError > 0)
//            Side.Pid.Output = abs(Side.Pid.Output);
//        else
//            Side.Pid.Output = -abs(Side.Pid.Output);

        //positive -> moves left
        MavCommand.linear.y = Side.Pid.Output;
        //ROS_INFO_STREAM("Pre " << ObjLocMsg.Xc << " Error " << newError << " U " << Side.Pid.Output);


        //********** FRONT

        //Front.Pid.Error = Front.References[MovIndex] - ObjLocMsg.YcPixel;
        //Front.Pid.Error = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
        newError = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
        Utils_MeanFilter(newError, Front.Pid.Error, PoseNetFilterSize);

        // integral term
        Front.Pid.Integral += Front.Pid.Error * Front.Pid.DeltaT;
        // clamp windup
        if(Front.Pid.Integral < -Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = -Front.Pid.IntegralWindupGuard;
        else if(Front.Pid.Integral > Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = Front.Pid.IntegralWindupGuard;

        // derivative term
        deltaError = Front.Pid.Error - Front.Pid.LastError;
        Front.Pid.Derivative = deltaError / Front.Pid.DeltaT;
        Front.Pid.LastError = Front.Pid.Error;

        // three stage control
        if(abs(Front.Pid.Error) < 3*ObjLocMsg.FrontThr)
        {
            Front.Pid.Output = 0;
            Front.Pid.Integral = 0;
        }
//        else if(abs(Front.Pid.Error) < 5*ObjLocMsg.FrontThr)
//        {
//            Front.Pid.Output = (Front.Pid.Kp * Front.Pid.Error)*0.1;
//        }
//        else if(abs(Front.Pid.Error) < 6*ObjLocMsg.FrontThr)
//        {
//            Front.Pid.Output = (Front.Pid.Kp * Front.Pid.Error) +
//                    (Front.Pid.Ki * Front.Pid.Integral) +
//                    (Front.Pid.Kd * Front.Pid.Derivative);
//        }
        else {
//            Front.Pid.Output = (Front.Pid.Kp * Front.Pid.Error);
//            Front.Pid.Integral = 0;
            Front.Pid.Output = (Front.Pid.Kp * Front.Pid.Error) +
                    (Front.Pid.Ki * Front.Pid.Integral) +
                    (Front.Pid.Kd * Front.Pid.Derivative);
        }

//        if(Front.Pid.Output < -0.02)
//            Front.Pid.Output = -0.02;
//        else if(Front.Pid.Output > 0.02)
//            Front.Pid.Output = 0.02;

        //positive -> moves front
        MavCommand.linear.x = Front.Pid.Output;
        //ROS_INFO_STREAM("Pre " << ObjLocMsg.Yc << " Error " << newError << " U " << Front.Pid.Output);



        if(abs(Side.Pid.Error) < 3*ObjLocMsg.SideThr &&
                abs(Front.Pid.Error) < 3*ObjLocMsg.FrontThr)
        {
            //********** ALTITUDE

            // Moves in meters, refs must be > 0
            //Altitude.Pid.Error = Altitude.References[MovIndex] - ObjLocMsg.Altitude;
            Altitude.Pid.Error = Altitude.References[MovIndex] - Altitude.Current;

            // integral term
            Altitude.Pid.Integral += Altitude.Pid.Error * Altitude.Pid.DeltaT;
            // clamp windup
            if(Altitude.Pid.Integral < -Altitude.Pid.IntegralWindupGuard)
                Altitude.Pid.Integral = -Altitude.Pid.IntegralWindupGuard;
            else if(Altitude.Pid.Integral > Altitude.Pid.IntegralWindupGuard)
                Altitude.Pid.Integral = Altitude.Pid.IntegralWindupGuard;

            // derivative term
            deltaError = Altitude.Pid.Error - Altitude.Pid.LastError;
            Altitude.Pid.Derivative = deltaError / Altitude.Pid.DeltaT;
            Altitude.Pid.LastError = Altitude.Pid.Error;

            if(abs(Altitude.Pid.Error) > 10*ObjLocMsg.AltThr)
            {
                Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error);
                Altitude.Pid.Integral = 0;
            }
            else {
                Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error) +
                        (Altitude.Pid.Ki * Altitude.Pid.Integral) +
                        (Altitude.Pid.Kd * Altitude.Pid.Derivative);
            }

            // positive -> moves up
            MavCommand.linear.z = Altitude.Pid.Output;



            //********** YAW

            // Referencia en grados
//#ifdef BEBOP2
            //Yaw.Pid.Error = Yaw.References[MovIndex] - ObjLocMsg.Yaw*180/M_PI;
//#else
            Yaw.Pid.Error = ObjLocMsg.Yaw*180/M_PI - Yaw.References[MovIndex]; // tum_simulator
//#endif

            // integral term
            Yaw.Pid.Integral += Yaw.Pid.Error * Yaw.Pid.DeltaT;
            // clamp windup
            if(Yaw.Pid.Integral < -Yaw.Pid.IntegralWindupGuard)
                Yaw.Pid.Integral = -Yaw.Pid.IntegralWindupGuard;
            else if(Yaw.Pid.Integral > Yaw.Pid.IntegralWindupGuard)
                Yaw.Pid.Integral = Yaw.Pid.IntegralWindupGuard;

            // derivative term
            deltaError = Yaw.Pid.Error - Yaw.Pid.LastError;
            Yaw.Pid.Derivative = deltaError / Yaw.Pid.DeltaT;
            Yaw.Pid.LastError = Yaw.Pid.Error;

            if(abs(Yaw.Pid.Error) > 3*ObjLocMsg.YawThr)
            {
                Yaw.Pid.Output = (Yaw.Pid.Kp * Yaw.Pid.Error);
                Yaw.Pid.Integral = 0;
            }
            else {
                Yaw.Pid.Output = (Yaw.Pid.Kp * Yaw.Pid.Error) +
                        (Yaw.Pid.Ki * Yaw.Pid.Integral) +
                        (Yaw.Pid.Kd * Yaw.Pid.Derivative);
            }

            // positive -> turns right?
            MavCommand.angular.z = Yaw.Pid.Output;



            if(abs(Yaw.Pid.Error) < 1*ObjLocMsg.YawThr &&
                    abs(Altitude.Pid.Error) < 5*ObjLocMsg.AltThr)
            {
                // Stop Yaw speed
                MavCommand.angular.z = 0;
                MavCommand.linear.x = 0;
                MavCommand.linear.y = 0;
                MavCommand.linear.z = 0;
                ROS_INFO_STREAM("REFERENCE REACHED");
                MsgStr = " REFERENCE REACHED ";
                MovState = MOV4_STANDBY;

                ROS_INFO_STREAM("Pose Bebop");
                ROS_INFO_STREAM(ViconBebopPose.transform.translation.x
                                << " " << ViconBebopPose.transform.translation.y
                                << " " << ViconBebopPose.transform.translation.z
                                << " " << ViconBebopPose.transform.rotation.w
                                << " " << ViconBebopPose.transform.rotation.x
                                << " " << ViconBebopPose.transform.rotation.y
                                << " " << ViconBebopPose.transform.rotation.z );
//                ROS_INFO_STREAM("Pose Bote");
//                ROS_INFO_STREAM(ViconBotePose.transform.translation.x
//                                << " " << ViconBotePose.transform.translation.y
//                                << " " << ViconBotePose.transform.translation.z
//                                << " " << ViconBotePose.transform.rotation.w
//                                << " " << ViconBotePose.transform.rotation.x
//                                << " " << ViconBotePose.transform.rotation.y
//                                << " " << ViconBotePose.transform.rotation.z );
                ROS_INFO_STREAM("ObjLoc");
                ROS_INFO_STREAM(ObjLocMsg.Altitude
                                << " " << ObjLocMsg.Yaw
                                << " " << ObjLocMsg.SF
                                << " " << ObjLocMsg.XcPixel
                                << " " << ObjLocMsg.YcPixel
                                << " " << ObjLocMsg.Q.w()
                                << " " << ObjLocMsg.Q.x()
                                << " " << ObjLocMsg.Q.y()
                                << " " << ObjLocMsg.Q.z() );
            }

        }
        else {
            MavCommand.angular.z = 0;
        }

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        //ROS_INFO_STREAM("Ux " << MavCommand.linear.x << " Uy " << MavCommand.linear.y << " Uz " << MavCommand.linear.z << " Uwz " << MavCommand.angular.z);

    } break;

    case MOV4_STANDBY: {
        // Mantener ultimo estado
        //MovIndex = 0;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        MavCommand.linear.z = 0;
    } break;

    default:
        break;
    } // end switch(MovState)

    // Update velocities
    MavCommandPub.publish(MavCommand);
    ROS_INFO_STREAM("Ux " << MavCommand.linear.x << " Uy " << MavCommand.linear.y << " Uz " << MavCommand.linear.z << " Uwz " << MavCommand.angular.z);
}


void ControlFrontal::PID_Control5(void) {
// Individual control for yaw, altitude, side and front velocities using
// ObjectLocalization Neural Network and pixel references.
// The NN outputs:
// Object's orientation (quaternion)
// Object's center (x,y) (pixels)
// Object's height, width (pixels)
// Object's confidence [0,1]
// Object's Scale Factor [-1,1], 0 -> 0.5m

    if(ObjLocMsg.Score > ObjLocMsg.ScoreThr)
        // add region, line of yaw
        ObjLoc_DrawBucketWithYaw(MavImg);

    if(AutoControl == 0) {
        cv::putText(MavImg, " CONTROLLER OFF ", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(255, 0, 0), 2, 2);
        cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
        MovState = MOV4_STANDBY;
        MovIndex = 0;
        imshow("DroneImage", MavImg);
        cv::waitKey(1);
        return;
    }

    ostringstream strYaw, strHeight;
    strHeight << ObjLocMsg.Altitude;
    strYaw << ObjLocMsg.Yaw * 180/M_PI;

    cv::putText(MavImg, " Yaw: " + strYaw.str() + " " + " Height: " + strHeight.str(), Point(5,20), FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0,0,255), 1, 2);
    cv::circle(MavImg, cv::Point(int(Side.References[MovIndex]), int(Front.References[MovIndex])), 10, cv::Scalar(0,0,255), 3);
    cv::putText(MavImg, " CONTROLLER ON ", Point(220, 340), FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 255, 0), 2, 2);
    imshow("DroneImage", MavImg);
    cv::waitKey(1);

    MovState = MOV4_SIDE;
    //bool noPrediction = false;
    if(ObjLocMsg.Score < ObjLocMsg.ScoreThr) {
        MovState = MOV4_STANDBY;
        //noPrediction = true;
    }

    switch(MovState) {
    case MOV4_SIDE: {
        // Moving side matches moving only on X axis
        // We work in pixels -> Refs must be > 0
        // in case no estimation is present
//        if(noPrediction) {
//            int tmpAdd = int(Side.Pid.Output * Side.Pid.DeltaT);
//            ObjLocMsg.Xc += tmpAdd;
//        }
        //********** SIDE

        //Side.Pid.Error = Side.References[MovIndex] - ObjLocMsg.XcPixel;
        //Side.Pid.Error = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        double newError = Side.References[MovIndex]/ObjLocMsg.ImgW - ObjLocMsg.Xc;
        Utils_MeanFilter(newError, Side.Pid.Error, PoseNetFilterSize);

        // integral term
        Side.Pid.Integral += Side.Pid.Error * Side.Pid.DeltaT;
        // clamp windup
        if(Side.Pid.Integral < -Side.Pid.IntegralWindupGuard)
            Side.Pid.Integral = -Side.Pid.IntegralWindupGuard;
        else if(Side.Pid.Integral > Side.Pid.IntegralWindupGuard)
            Side.Pid.Integral = Side.Pid.IntegralWindupGuard;

        // derivative term
        double deltaError = Side.Pid.Error - Side.Pid.LastError;
        Side.Pid.Derivative = deltaError / Side.Pid.DeltaT;
        Side.Pid.LastError = Side.Pid.Error;

        // three stage control
        if(abs(Side.Pid.Error) > 5*ObjLocMsg.SideThr)
        {
            Side.Pid.Output = (Side.Pid.Kp * Side.Pid.Error);
            Side.Pid.Integral = 0;
        }
//        else if(abs(Side.Pid.Error) > 3*ObjLocMsg.SideThr) {
//            Side.Pid.Output = (Side.Pid.Kp * Side.Pid.Error) +
//                    (Side.Pid.Ki * Side.Pid.Integral) +
//                    (Side.Pid.Kd * Side.Pid.Derivative);
//        }
        else {
//            Side.Pid.Output = 0;
//            Side.Pid.Integral = 0;
            Side.Pid.Output = (Side.Pid.Kp * Side.Pid.Error) +
                    (Side.Pid.Ki * Side.Pid.Integral) +
                    (Side.Pid.Kd * Side.Pid.Derivative);
        }
//        Side.Pid.Output = (Side.Pid.Kp * Side.Pid.Error) +
//                (Side.Pid.Ki * Side.Pid.Integral) +
//                (Side.Pid.Kd * Side.Pid.Derivative);

        //positive -> moves left
        MavCommand.linear.y = Side.Pid.Output;


        //********** FRONT

        //Front.Pid.Error = Front.References[MovIndex] - ObjLocMsg.YcPixel;
        //Front.Pid.Error = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
        newError = Front.References[MovIndex]/ObjLocMsg.ImgH - ObjLocMsg.Yc;
        Utils_MeanFilter(newError, Front.Pid.Error, PoseNetFilterSize);

        // integral term
        Front.Pid.Integral += Front.Pid.Error * Front.Pid.DeltaT;
        // clamp windup
        if(Front.Pid.Integral < -Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = -Front.Pid.IntegralWindupGuard;
        else if(Front.Pid.Integral > Front.Pid.IntegralWindupGuard)
            Front.Pid.Integral = Front.Pid.IntegralWindupGuard;

        // derivative term
        deltaError = Front.Pid.Error - Front.Pid.LastError;
        Front.Pid.Derivative = deltaError / Front.Pid.DeltaT;
        Front.Pid.LastError = Front.Pid.Error;

        // three stage control
        if(abs(Front.Pid.Error) > 5*ObjLocMsg.FrontThr)
        {
            Front.Pid.Output = (Front.Pid.Kp * Front.Pid.Error);
            Front.Pid.Integral = 0;
        }
//        else if(abs(Front.Pid.Error) > 3*ObjLocMsg.FrontThr) {
//            Front.Pid.Output = (Front.Pid.Kp * Front.Pid.Error) +
//                    (Front.Pid.Ki * Front.Pid.Integral) +
//                    (Front.Pid.Kd * Front.Pid.Derivative);
//        }
        else {
//            Front.Pid.Output = 0;
//            Front.Pid.Integral = 0;
            Front.Pid.Output = (Front.Pid.Kp * Front.Pid.Error) +
                    (Front.Pid.Ki * Front.Pid.Integral) +
                    (Front.Pid.Kd * Front.Pid.Derivative);
        }
//        Front.Pid.Output = (Front.Pid.Kp * Front.Pid.Error) +
//                (Front.Pid.Ki * Front.Pid.Integral) +
//                (Front.Pid.Kd * Front.Pid.Derivative);

        //positive -> moves front
        MavCommand.linear.x = Front.Pid.Output;


        //********** ALTITUDE

//        // Moves in meters, refs must be > 0
//        Altitude.Pid.Error = Altitude.References[MovIndex] - ObjLocMsg.Deep;

//        // integral term
//        Altitude.Pid.Integral += Altitude.Pid.Error * Altitude.Pid.DeltaT;
//        // clamp windup
//        if(Altitude.Pid.Integral < -Altitude.Pid.IntegralWindupGuard)
//            Altitude.Pid.Integral = -Altitude.Pid.IntegralWindupGuard;
//        else if(Altitude.Pid.Integral > Altitude.Pid.IntegralWindupGuard)
//            Altitude.Pid.Integral = Altitude.Pid.IntegralWindupGuard;

//        // derivative term
//        deltaError = Altitude.Pid.Error - Altitude.Pid.LastError;
//        Altitude.Pid.Derivative = deltaError / Altitude.Pid.DeltaT;
//        Altitude.Pid.LastError = Altitude.Pid.Error;

//        if(abs(Altitude.Pid.Error) > 10*ObjLocMsg.AltThr)
//        {
//            Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error);
//            Altitude.Pid.Integral = 0;
//        }
//        else {
//            Altitude.Pid.Output = (Altitude.Pid.Kp * Altitude.Pid.Error) +
//                    (Altitude.Pid.Ki * Altitude.Pid.Integral) +
//                    (Altitude.Pid.Kd * Altitude.Pid.Derivative);
//        }

//        // positive -> moves up
//        MavCommand.linear.z = Altitude.Pid.Output;


//        if(abs(Side.Pid.Error) < 3*ObjLocMsg.SideThr &&
//                abs(Front.Pid.Error) < 3*ObjLocMsg.FrontThr)
//        {
//            //ROS_INFO_STREAM("SIDE FRONT COMPLETED");

//            //********** YAW

//            // Referencia en grados
////#ifdef BEBOP2
////            Yaw.Pid.Error = Yaw.References[MovIndex] - ObjLocMsg.Yaw*180/M_PI;
////#else
//            Yaw.Pid.Error = ObjLocMsg.Yaw*180/M_PI - Yaw.References[MovIndex]; // tum_simulator
////#endif

//            // integral term
//            Yaw.Pid.Integral += Yaw.Pid.Error * Yaw.Pid.DeltaT;
//            // clamp windup
//            if(Yaw.Pid.Integral < -Yaw.Pid.IntegralWindupGuard)
//                Yaw.Pid.Integral = -Yaw.Pid.IntegralWindupGuard;
//            else if(Yaw.Pid.Integral > Yaw.Pid.IntegralWindupGuard)
//                Yaw.Pid.Integral = Yaw.Pid.IntegralWindupGuard;

//            // derivative term
//            deltaError = Yaw.Pid.Error - Yaw.Pid.LastError;
//            Yaw.Pid.Derivative = deltaError / Yaw.Pid.DeltaT;
//            Yaw.Pid.LastError = Yaw.Pid.Error;

//            if(abs(Yaw.Pid.Error) > 2*ObjLocMsg.YawThr)
//            {
//                Yaw.Pid.Output = (Yaw.Pid.Kp * Yaw.Pid.Error);
//                Yaw.Pid.Integral = 0;
//            }
//            else {
//                Yaw.Pid.Output = (Yaw.Pid.Kp * Yaw.Pid.Error) +
//                        (Yaw.Pid.Ki * Yaw.Pid.Integral) +
//                        (Yaw.Pid.Kd * Yaw.Pid.Derivative);
//            }

//            // positive -> turns right?
//            MavCommand.angular.z = Yaw.Pid.Output;


////            // Referencia en grados
////            // 90° y -90° parecen generan conflicto
////            double tmpYaw = ObjLocMsg.Yaw*180/M_PI;
//////            if(tmpYaw < -89 && tmpYaw > -91)
//////                ObjLocMsg.Yaw = M_PI/2;

//////#ifdef BEBOP2
////            Yaw.Pid.Error = Yaw.References[MovIndex] - ObjLocMsg.Yaw*180/M_PI;
//////#else
//////            Yaw.Pid.Error = ObjLocMsg.Yaw*180/M_PI - Yaw.References[MovIndex]; // tum_simulator
//////#endif
////            //~ ROS_INFO_STREAM("yaw_reference = " << yaw_reference[index]);

////            // Controlador en SO3, solo en yaw, en radianes
////            // Posicion actual
////            //Yaw.Pid.Rt.setEulerYPR(ObjLocMsg.Yaw*M_PI/180, 0, 0);
////            Yaw.Pid.Rt.setEulerYPR(ObjLocMsg.Yaw, 0, 0);
////            // Posicion destino
////            Yaw.Pid.Rd.setEulerYPR(Yaw.References[MovIndex]*M_PI/180, 0, 0);

////            // Re = Rd^T * R(t)
////            tf::Matrix3x3 Re = Yaw.Pid.Rd.transposeTimes(Yaw.Pid.Rt);
////            double y[3];
////            // y = [h-f, c-g, d-b]^T, ejes [x,y,z]
////            y[0] = Re[2].getY() - Re[1].getZ();
////            y[1] = Re[0].getZ() - Re[2].getX();
////            y[2] = Re[1].getX() - Re[0].getY();

////            // norma de y, |y|
////            double y_n;
////            y_n = sqrt(y[0]*y[0] + y[1]*y[1] + y[2]*y[2]);
////            //ROS_INFO_STREAM("y_n = " << y_n);
////            // control only for yaw
////            if (y_n == 0.0)
////                y_n = 1;
////            else if (y_n > 1)
////                y_n = 1;
////            else if (y_n < -1)
////                y_n = -1;

////            Yaw.Pid.Output = (-1*Yaw.Pid.Kp*asin(y_n) / y_n) * y[2];

////            //ROS_INFO_STREAM("Yaw Ref = " << Yaw.Reference[SeqIndex]);
////            //ROS_INFO_STREAM("Yaw U: " << Yaw.Pid.U);

////            if(tmpYaw < 10)
////                Yaw.Pid.Output = -abs(Yaw.Pid.Output);
////            else if(tmpYaw > 170)
////                Yaw.Pid.Output = abs(Yaw.Pid.Output);
////            MavCommand.angular.z = Yaw.Pid.Output;


//            if(abs(Yaw.Pid.Error) < ObjLocMsg.YawThr &&
//                    abs(Altitude.Pid.Error) < ObjLocMsg.AltThr)
//            {
//                // Stop Yaw speed
//                MavCommand.angular.z = 0;
//                ROS_INFO_STREAM("REFERENCE REACHED");
//                ROS_INFO_STREAM(PoseX
//                                << " " << PoseY
//                                << " " << PoseZ
//                                << " " << GazeboPose.orientation.w
//                                << " " << GazeboPose.orientation.x
//                                << " " << GazeboPose.orientation.y
//                                << " " << GazeboPose.orientation.z
//                                << " " << ObjLocMsg.Deep
//                                << " " << ObjLocMsg.Q.w()
//                                << " " << ObjLocMsg.Q.x()
//                                << " " << ObjLocMsg.Q.y()
//                                << " " << ObjLocMsg.Q.z() );
//            }
//        }
//        else {
//            // Stop Yaw speed
//            MavCommand.angular.z = 0;
//        }
    } break;

    case MOV4_STANDBY: {
        // Mantener ultimo estado
        //MovIndex = 0;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        MavCommand.linear.z = 0;

        //MovState = MOV4_SELECT;
    } break;

    default:
        break;
    } // end switch(MovState)

//    ROS_INFO_STREAM(MavCommand.angular.x
//                    << " " << MavCommand.angular.y
//                    << " " << MavCommand.angular.z
//                    << " " << MavCommand.linear.x
//                    << " " << MavCommand.linear.y
//                    << " " << MavCommand.linear.z);

    // Update velocities
    MavCommandPub.publish(MavCommand);
}


void ControlFrontal::ObjLoc_DrawGrasper(cv::Mat &img) {
    // Formats region and orientation with predicted values

//    double width = img.size().width; // src.cols
//    double height = img.size().height; // src.rows
    
//    ObjLocGrasperMsg.ImgH = height;
//    ObjLocGrasperMsg.ImgW = width;

    double height = ObjGrasperMsg.ImgH;
    double width = ObjGrasperMsg.ImgW;

//    IMAGE
    double x0 = width * (ObjGrasperMsg.Xc - ObjGrasperMsg.W / 2);
    double y0 = height * (ObjGrasperMsg.Yc - ObjGrasperMsg.H / 2);
    double x1 = x0 + width * ObjGrasperMsg.W;
    double y1 = y0 + height * ObjGrasperMsg.H;
//    print('Score: {:.4f}'.format(score))

//    cv2.rectangle(unscaled, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)
    cv::rectangle(img, cv::Point(int(x0), int(y0)), cv::Point(int(x1), int(y1)), cv::Scalar(0, 0, 255), 3);
}


void ControlFrontal::ObjLoc_DrawBucket(cv::Mat &img) {
    // Formats region and orientation with predicted values

//    double width = img.size().width; // src.cols
//    double height = img.size().height; // src.rows

//    ObjLocMsg.ImgH = height;
//    ObjLocMsg.ImgW = width;

    double height = ObjLocMsg.ImgH;
    double width = ObjLocMsg.ImgW;

//    IMAGE
    double x0 = width * (ObjLocMsg.Xc - ObjLocMsg.W / 2);
    double y0 = height * (ObjLocMsg.Yc - ObjLocMsg.H / 2);
    double x1 = x0 + width * ObjLocMsg.W;
    double y1 = y0 + height * ObjLocMsg.H;
//    print('Score: {:.4f}'.format(score))

//    cv2.rectangle(unscaled, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)
    cv::rectangle(img, cv::Point(int(x0), int(y0)), cv::Point(int(x1), int(y1)), cv::Scalar(0, 255, 0), 3);
}


void ControlFrontal::ObjLoc_DrawBucketWithYaw(cv::Mat &img) {
    // Formats region and orientation with predicted values

//    double width = img.size().width; // src.cols
//    double height = img.size().height; // src.rows
    
//    ObjLocMsg.ImgH = height;
//    ObjLocMsg.ImgW = width;

    double height = ObjLocMsg.ImgH;
    double width = ObjLocMsg.ImgW;

//    IMAGE
    double x0 = width * (ObjLocMsg.Xc - ObjLocMsg.W / 2);
    double y0 = height * (ObjLocMsg.Yc - ObjLocMsg.H / 2);
    double x1 = x0 + width * ObjLocMsg.W;
    double y1 = y0 + height * ObjLocMsg.H;
//    print('Score: {:.4f}'.format(score))

//    cv2.rectangle(unscaled, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 3)
    cv::rectangle(img, cv::Point(int(x0), int(y0)), cv::Point(int(x1), int(y1)), cv::Scalar(0, 255, 0), 3);

//    #q0, q1, q2, q3 = poses[ selected_indices[i] ]
//    q = np.asarray([q0,q1,q2,q3])
//    print('Q Pred. W {:.4f} X {:.4f} Y {:.4f} Z {:.4f}'.format(q[0],q[1],q[2],q[3]))
//    q = q / np.linalg.norm(q)
//    print('Q Pred. W {:.4f} X {:.4f} Y {:.4f} Z {:.4f}'.format(q[0],q[1],q[2],q[3]))
//    q = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
//    angles = Q_ToRPY(q)
//    print('Angles Pred. R {:.4f} P {:.4f} Y {:.4f}'.format(angles.Roll*180/math.pi, angles.Pitch*180/math.pi, angles.Yaw*180/math.pi))

//    pc_x = int(x0 + (x1-x0)/2)
//    pc_y = int(y0 + (y1-y0)/2)
//    p_x = pc_x + 50
//    p_y = pc_y
//    a = angles.Yaw
//    p_rot = RotatePoint((pc_x, pc_y), (p_x, p_y) , a)
//    cv2.line(unscaled,(pc_x, pc_y),p_rot,(0,255,0),3)
//    p_rot = RotatePoint((pc_x, pc_y), (p_x, p_y) , math.pi + a)
//    cv2.line(unscaled,(pc_x, pc_y),p_rot,(0,255,0),3)
    int pc_x = int(x0 + (x1-x0)/2);
    int pc_y = int(y0 + (y1-y0)/2);
    int p_x = pc_x + 50;
    int p_y = pc_y;
    cv::Point p_rot = cv::Point(0, 0);
    ObjLoc_RotatePoint(p_rot, cv::Point(pc_x, pc_y), cv::Point(p_x, p_y), ObjLocMsg.Yaw);
    cv::line(img, cv::Point(pc_x, pc_y), p_rot, cv::Scalar(0,255,0), 3);
    ObjLoc_RotatePoint(p_rot, cv::Point(pc_x, pc_y), cv::Point(p_x, p_y), M_PI + ObjLocMsg.Yaw);
    cv::line(img, cv::Point(pc_x, pc_y), p_rot, cv::Scalar(0,255,0), 3);
}


void ControlFrontal::ObjLoc_RotatePoint(cv::Point &p_rot, cv::Point pc, cv::Point p, double a) {
    // Rotates point 'p' 'a' radians over center point 'pc'.

    int x0 = pc.x; //[0];
    int y0 = pc.y; //[1]
    int x1 = p.x; //[0]
    int y1 = p.y; //[1]

    a = -1*a; // or 2*math.pi - a, OpenCV CCW

    double x2 = (double(x1 - x0) * cos(a)) - (double(y1 - y0) * sin(a)) + double(x0);
    double y2 = (double(x1 - x0) * sin(a)) + (double(y1 - y0) * cos(a)) + double(y0);

    p_rot.x = int(x2);
    p_rot.y = int(y2);
}


void ControlFrontal::ObjLoc_GrasperCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    // Receives msg=[y_c, x_c, h, w, score].

    ObjGrasperMsg.Score = double(msg->data[4]);
    if(ObjGrasperMsg.Score < ObjGrasperMsg.ScoreThr) {
        ObjGrasperMsg.Score = 0;
        return;
    }

    ObjGrasperMsg.H = double(msg->data[2]);
    ObjGrasperMsg.W = double(msg->data[3]);
    // Avoid NaNs, do not update sensitive data to zeros
    ObjGrasperMsg.Yc = double(msg->data[0]); //const_cast<double>(msg->data[0]);
    ObjGrasperMsg.Xc = double(msg->data[1]);

    //ROS_INFO_STREAM("Yc " << ObjLocGrasperMsg.Yc << " Xc " << ObjLocGrasperMsg.Xc << " H " << ObjLocGrasperMsg.H << " W " << ObjLocGrasperMsg.W << " Score " << ObjLocGrasperMsg.Score);

    // Smooth data
    //ObjGrasperMsg.Xc = Utils_MeanFilter(ObjGrasperMsg.Xc, ObjGrasperMsg.XcFiltered, 5);
    //ObjGrasperMsg.Yc = Utils_MeanFilter(ObjGrasperMsg.Yc, ObjGrasperMsg.YcFiltered, 5);
    Utils_KalmanUpdate(ObjLocMsg.XcKalman, ObjLocMsg.Xc);
    Utils_KalmanUpdate(ObjLocMsg.YcKalman, ObjLocMsg.Yc);
    
    // To pixels
    ObjGrasperMsg.XcPixel = int(ObjGrasperMsg.Xc * ObjGrasperMsg.ImgW);
    ObjGrasperMsg.YcPixel = int(ObjGrasperMsg.Yc * ObjGrasperMsg.ImgH);
}


void ControlFrontal::ObjLoc_BucketCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    // Receives msg=[y_c, x_c, h, w, qW, qX, qY, qZ, SF, score].
    // Object rotation and deep (height to object) are updated only if
    // prediction has a good score.

    ObjLocMsg.Score = double(msg->data[9]);
    if(ObjLocMsg.Score < ObjLocMsg.ScoreThr) {
        ObjLocMsg.Score = 0;
        return;
    }

    ObjLocMsg.H = double(msg->data[2]);
    ObjLocMsg.W = double(msg->data[3]);
    // Avoid NaNs, do not update sensitive data to zeros
//    if(ObjLocMsg.Score > ObjLocMsg.ScoreThr) {
        ObjLocMsg.Yc = double(msg->data[0]); //const_cast<double>(msg->data[0]);
        ObjLocMsg.Xc = double(msg->data[1]);

        ObjLocMsg.SF = double(msg->data[8]);
        ObjLocMsg.Q.setValue(double(msg->data[5]), double(msg->data[6]), double(msg->data[7]), double(msg->data[4]));
        ObjLocMsg.Q.normalize();
//    }


    //ROS_INFO_STREAM("Yc " << ObjLocMsg.Yc << " Xc " << ObjLocMsg.Xc << " H " << ObjLocMsg.H << " W " << ObjLocMsg.W << " Score " << ObjLocMsg.Score);
    //ROS_INFO_STREAM("qW " << ObjLocMsg.Q.w() << " qX " << ObjLocMsg.Q.x() << " qY " << ObjLocMsg.Q.y() << " qZ " << ObjLocMsg.Q.z());

    tf::Matrix3x3 m(ObjLocMsg.Q);
    m.getRPY(ObjLocMsg.Roll, ObjLocMsg.Pitch, ObjLocMsg.Yaw);
    //Utils_Q2Y(ObjLocMsg.Q, ObjLocMsg.Yaw);
    //ObjLocMsg.Roll = (ObjLocMsg.Roll * 180) / M_PI;
    //ObjLocMsg.Pitch = (ObjLocMsg.Pitch * 180) / M_PI;
    //ObjLocMsg.Yaw = (ObjLocMsg.Yaw * 180) / M_PI;

    // Avoid negative angles
    if(ObjLocMsg.Yaw < 0)
        ObjLocMsg.Yaw = M_PI - abs(ObjLocMsg.Yaw);
    
    if(ObjLocMsg.YawFiltered > 160*M_PI/180 && ObjLocMsg.Yaw >= 0 && ObjLocMsg.Yaw < 20*M_PI/180)
        ObjLocMsg.Yaw = M_PI + abs(ObjLocMsg.Yaw);
    else if(ObjLocMsg.YawFiltered < 20*M_PI/180 && ObjLocMsg.Yaw <= M_PI && ObjLocMsg.Yaw > 160*M_PI/180)
        ObjLocMsg.Yaw = ObjLocMsg.Yaw - M_PI;
    


    // 20°,160° vs 45°,135°
//    if(ObjLocMsg.YawFiltered >= 20*M_PI/180 && ObjLocMsg.Yaw < 0)
//        ObjLocMsg.Yaw = M_PI - abs(ObjLocMsg.Yaw);
//    else if(ObjLocMsg.YawFiltered > 160*M_PI/180 && ObjLocMsg.Yaw < 20*M_PI/180)
//        ObjLocMsg.Yaw = M_PI - abs(ObjLocMsg.Yaw);
    //if(ObjLocMsg.YawFiltered > 160*M_PI/180 && ObjLocMsg.Yaw < 20*M_PI/180)
    //    ObjLocMsg.Yaw = M_PI - abs(ObjLocMsg.Yaw);


    // Smooth data
    //ObjLocMsg.Xc = Utils_MeanFilter(ObjLocMsg.Xc, ObjLocMsg.XcFiltered, PoseNetFilterSize);
    //ObjLocMsg.Yc = Utils_MeanFilter(ObjLocMsg.Yc, ObjLocMsg.YcFiltered, PoseNetFilterSize);
    Utils_MeanFilter(ObjLocMsg.Yaw, ObjLocMsg.YawFiltered, 20);
//    if(ObjLocMsg.YawFiltered < 0)
//        ObjLocMsg.YawFiltered = 0; //ObjLocMsg.YawFiltered = abs(ObjLocMsg.YawFiltered);
    ObjLocMsg.Yaw = ObjLocMsg.YawFiltered;

    ObjLocMsg.SF = Utils_MeanFilter(ObjLocMsg.SF, ObjLocMsg.SFFiltered, 50);

    // Scale Factor to meters
    ObjLocMsg.Altitude = 0.5/(ObjLocMsg.SF+1);
    
    // To pixels
    ObjLocMsg.XcPixel = int(ObjLocMsg.Xc * ObjLocMsg.ImgW);
    ObjLocMsg.YcPixel = int(ObjLocMsg.Yc * ObjLocMsg.ImgH);

    //ROS_INFO_STREAM("ObjRoll " << ObjLocMsg.Roll << " ObjPitch " << Pitch << " Yaw " << Yaw.Current << " qZ " << ObjLocMsg.Q.z());
    //ROS_INFO("Obj Roll: %.2f Pitch: %.2f Yaw: %.2f\n Height: %.2f\n", ObjLocMsg.Roll*180/M_PI, ObjLocMsg.Pitch*180/M_PI, ObjLocMsg.Yaw*180/M_PI, 0.5/(ObjLocMsg.SF+1));
    //ROS_INFO("Obj Roll: %.2f Pitch: %.2f Yaw: %.2f\n Height: %.2f\n", ObjLocMsg.Roll, ObjLocMsg.Pitch, ObjLocMsg.Yaw, ObjLocMsg.Deep);
}


void ControlFrontal::Utils_Q2Y(tf::Quaternion &q, double &yaw)
{
    // Return Roll, Pitch, Yaw in radians

//    angles = EulerAngles(0, 0, 0)

//    // roll (x-axis rotation)
//    sinr_cosp = 2.0 * (q.W * q.X + q.Y * q.Z);
//    cosr_cosp = 1.0 - 2.0 * (q.X * q.X + q.Y * q.Y);
//    angles.Roll = math.atan2(sinr_cosp, cosr_cosp);

//    // pitch (y-axis rotation)
//    sinp = 2.0 * (q.W * q.Y - q.Z * q.X);
//    if math.fabs(sinp) >= 1:
//        angles.Pitch = math.copysign(math.pi / 2, sinp); // use 90 degrees if out of range
//    else:
//        angles.Pitch = math.asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}


void ControlFrontal::Utils_KalmanInit(struct Kalman_ &kalman, double x00, double p00, double q, double r1)
{   // Kalman's initial values.
    // https://www.kalmanfilter.net/kalman1d.html

    kalman.X00 = x00;
    kalman.X10 = x00;
    kalman.P00 = p00;
    kalman.P10 = p00 + q;
    kalman.Q = q;
    kalman.R1 = r1;
}


void ControlFrontal::Utils_KalmanUpdate(struct Kalman_ &kalman, double z1)
{
    // Updates state X11 of Kalman Filter. X11 is predicted (filtered) value.
    // X00 initial system state
    // P00 initial state uncertainty (covariance)
    // Zn measured system state
    // Rn measurement uncertainty (covariance)
    // Xnn system state estimate
    // Pnn estimate uncertainty
    // Q??
    // https://www.kalmanfilter.net/kalman1d.html

    //    x00 = 0.02
        //x10 = x00
    //    p00 = 0.1*0.1
    //    q = 0.0
        //p10 = p00 + q
    //    r1 = 0.03*0.03

        // mesaure
    //    z1 = results[i,0]
        // update
        kalman.K = kalman.P10 / (kalman.P10 + kalman.R1);
        kalman.X11 = kalman.X10 + kalman.K*(z1 - kalman.X10);
        kalman.P11 = (1 - kalman.K)*kalman.P10;
    //    filtered_k[i] = x11
        // predict
        kalman.X10 = kalman.X11;
        kalman.P10 = kalman.P11 + kalman.Q;

        //return kalman.x11
}


double ControlFrontal::Utils_MeanFilter(double newVal, double &mean_1, double filterSize)
{
    mean_1 = (1 - 1/filterSize)*mean_1 + (1/filterSize)*newVal;
    return mean_1;
}


double ControlFrontal::PoseNet_AverageFilter(double newVal, std::vector<double> &buffer, uint32_t filterSize)
{
    if(buffer.size() == filterSize) {
        buffer.erase(buffer.begin()); //pop_front();
        buffer.push_back(newVal);

        double sum = 0;
        for(uint32_t i = 0; i < filterSize; i++) {
            sum += buffer[i];
        }
        return sum / filterSize;
    }
    else {
        buffer.push_back(newVal);
        return 0;
    }
}


void ControlFrontal::Vicon_BebopCallback(const geometry_msgs::TransformStamped::ConstPtr &pose) {
    ViconBebopPose.transform.rotation.w = pose->transform.rotation.w;
    ViconBebopPose.transform.rotation.x = pose->transform.rotation.x;
    ViconBebopPose.transform.rotation.y = pose->transform.rotation.y;
    ViconBebopPose.transform.rotation.z = pose->transform.rotation.z;

    ViconBebopPose.transform.translation.x = pose->transform.translation.x;
    ViconBebopPose.transform.translation.y = pose->transform.translation.y;
    ViconBebopPose.transform.translation.z = pose->transform.translation.z;
}


void ControlFrontal::Vicon_BoteCallback(const geometry_msgs::TransformStamped::ConstPtr &pose) {
    ViconBotePose.transform.rotation.w = pose->transform.rotation.w;
    ViconBotePose.transform.rotation.x = pose->transform.rotation.x;
    ViconBotePose.transform.rotation.y = pose->transform.rotation.y;
    ViconBotePose.transform.rotation.z = pose->transform.rotation.z;

    ViconBotePose.transform.translation.x = pose->transform.translation.x;
    ViconBotePose.transform.translation.y = pose->transform.translation.y;
    ViconBotePose.transform.translation.z = pose->transform.translation.z;
}


void ControlFrontal::PoseNet_Callback(const geometry_msgs::Pose::ConstPtr &pose) {
    PoseX = GazeboPose.position.x = pose->position.x;
    PoseY = GazeboPose.position.y = pose->position.y;
    PoseZ = GazeboPose.position.z = pose->position.z;
    GazeboPose.orientation.x = pose->orientation.x;
    GazeboPose.orientation.y = pose->orientation.y;
    GazeboPose.orientation.z = pose->orientation.z;
    GazeboPose.orientation.w = pose->orientation.w;

    tf::Quaternion q(GazeboPose.orientation.x, GazeboPose.orientation.y, GazeboPose.orientation.z, GazeboPose.orientation.w);
    q.normalize();
    GazeboPose.orientation.x = q.x();
    GazeboPose.orientation.y = q.y();
    GazeboPose.orientation.z = q.z();
    GazeboPose.orientation.w = q.w();

    PoseX = PoseNet_AverageFilter(PoseX, Front.References, uint32_t(PoseNetFilterSize));
    PoseY = PoseNet_AverageFilter(PoseY, Side.References, uint32_t(PoseNetFilterSize));
    PoseZ = PoseNet_AverageFilter(PoseZ, Altitude.References, uint32_t(PoseNetFilterSize));

    ROS_INFO_STREAM("X: " << PoseX << " Y: " << PoseY << " Z: " << PoseZ);
}


void ControlFrontal::Gazebo_Pose_Callback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
    PoseX = GazeboPose.position.x = msg->pose[ulong(GazeboModelID)].position.x;
    PoseY = GazeboPose.position.y = msg->pose[ulong(GazeboModelID)].position.y;
    PoseZ = GazeboPose.position.z = msg->pose[ulong(GazeboModelID)].position.z;
    GazeboPose.orientation.x = msg->pose[ulong(GazeboModelID)].orientation.x;
    GazeboPose.orientation.y = msg->pose[ulong(GazeboModelID)].orientation.y;
    GazeboPose.orientation.z = msg->pose[ulong(GazeboModelID)].orientation.z;
    GazeboPose.orientation.w = msg->pose[ulong(GazeboModelID)].orientation.w;

    //ROS_INFO_STREAM("Gazebo pose: X = " << start_pose.position.x << " Y = " << start_pose.position.y << " Z = " << start_pose.position.z );
}


//~ void ControlFrontal::Ardrone_Altitude_Callback(const sensor_msgs::Range::ConstPtr &range)
void ControlFrontal::Ardrone_Altitude_Callback(const bebop_msgs::Ardrone3PilotingStateAltitudeChanged &alt)
{
    Altitude.Current = alt.altitude;
}


void ControlFrontal::Bebop2_Odom_Callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    ARdroneOdomQx = odom->pose.pose.orientation.x;
    ARdroneOdomQy = odom->pose.pose.orientation.y;
    ARdroneOdomQz = odom->pose.pose.orientation.z;
    ARdroneOdomQw = odom->pose.pose.orientation.w;

    //Q.setValue(odom_ARdrone_x,odom_ARdrone_y,odom_ARdrone_z,odom_ARdrone_w);
    tf::Quaternion q1(ARdroneOdomQx, ARdroneOdomQy, ARdroneOdomQz, ARdroneOdomQw);
    //tf::Matrix3x3 m(Q);
    tf::Matrix3x3 m(q1);

    m.getRPY(Roll, Pitch, Yaw.Current);

    Roll = (Roll * 180) / M_PI;
    Pitch = (Pitch * 180) / M_PI;
    Yaw.Current = (Yaw.Current * 180) / M_PI;
    //ROS_INFO("Yaw: %.2f Pitch: %.2f Roll: %.2f\n", Roll, Pitch, Yaw.Current);
}


void ControlFrontal::Ardrone_Odom_Callback(const sensor_msgs::Imu::ConstPtr &odom)
{	
    ARdroneOdomQx = odom->orientation.x;
    ARdroneOdomQy = odom->orientation.y;
    ARdroneOdomQz = odom->orientation.z;
    ARdroneOdomQw = odom->orientation.w;

    //Q.setValue(odom_ARdrone_x,odom_ARdrone_y,odom_ARdrone_z,odom_ARdrone_w);
    tf::Quaternion q1(ARdroneOdomQx, ARdroneOdomQy, ARdroneOdomQz, ARdroneOdomQw);
    //tf::Matrix3x3 m(Q);
    tf::Matrix3x3 m(q1);

    m.getRPY(Roll, Pitch, Yaw.Current);

    Roll = (Roll * 180) / M_PI;
    Pitch = (Pitch * 180) / M_PI;
    Yaw.Current = (Yaw.Current * 180) / M_PI;
}


void ControlFrontal::Keyboard_Callback(const std_msgs::Int8 &flag) {
    if(flag.data == 6) {
        AutoControl = 1;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        MavCommand.linear.z = 0;
        MavCommandPub.publish(MavCommand);
    }

    if(flag.data == 10) {
        AutoControl = 0;

        Traj.TimesCnt = 0;
        Traj.XCnt = 0;
        Traj.YCnt = 0;
        Traj.ZCnt = 0;
        Traj.Yaw = 0;
        Traj.State = 0;
        SaveOn = false;
        ModelToSetPose.position.x = Traj.XIni;
        ModelToSetPose.position.y = Traj.YIni;
        ModelToSetPose.position.z = Traj.ZIni;

        MavCommand.angular.x = 0;
        MavCommand.angular.y = 0;
        MavCommand.angular.z = 0;
        MavCommand.linear.x = 0;
        MavCommand.linear.y = 0;
        MavCommand.linear.z = 0;
        MavCommandPub.publish(MavCommand);

        ImageCount = Traj.DiscardFrames;
        DataLine.clear();
    }

    ROS_INFO_STREAM("AutoControl: " << AutoControl);
}


void ControlFrontal::Utils_ImageCenterCrop(cv::Mat &img, int outW, int outH) {
    // Crops an image considering that out.size < img.size

    int width = img.size().width; // src.cols
    int height = img.size().height; // src.rows

    // Setup a rectangle to define your region of interest
    int tpX = int( (width - outW)/2 );
    int tpY = int( (height - outH)/2 ) + 60; // 60px offset to get bottom ROI
    cv::Rect myROI(tpX, tpY, outW, outH);
    // Crop the full image to that image contained by the rectangle myROI
    // Note that this doesn't copy the data
    img = img(myROI);
}


double ControlFrontal::Utils_AngleBetweenLines(int x1, int y1, int x2, int y2) {
    // Calculates angle = acos(A dot B / |A||B|) where A,B e R^2 over 360°
    // center is at the current reference coordenates (in pixels).
    // TODO debug initial point.

    double angle;

    double dotAB = x1*x2 + y1*y2;
    double normA = sqrt(x1*x1 + y1*y1);
    double normB = sqrt(x2*x2 + y2*y2);
    angle = acos(dotAB / (normA * normB));

    // to degrees
    angle = angle * 180 / M_PI;

    // for static line (x,y)=(1,0)
//    if(y2 < 0)
//        angle = 360 - angle;

    // for static line (x,y)=(0,1)
    if(x2 > 0)
        angle = 360 - angle;

    return angle;
}
