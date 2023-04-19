#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "md/vel_msg.h"
#include "md/monitor_msg.h"

#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <sensor_msgs/JointState.h>
//
typedef unsigned char  BYTE;

#define LEFT           	  0
#define RIGHT             1

#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80

#define DEG2RAD(x) (x * 0.01745329252) // *PI/180
#define RAD2DEG(x) (x * 57.2957795131) // *180/PI

bool debug_print=false;
//int not_moving_count=0;
double mct;
const double Pi = 3.14159265359;

typedef struct {
    long lPosiX;
    long lPosiY, lExPosiY;
    short sTheta;
    float fTheta;
    short sVoltIn;
    short sCurrent[2];
    BYTE byUS1;
    BYTE byUS2;
    BYTE byUS3;
    BYTE byUS4;
    BYTE byPlatStatus;
    BYTE byDocStatus;
    BYTE byMotStatus[2];

    BYTE fgAlarm[2], fgCtrlFail[2], fgOverVolt[2], fgOverTemp[2];
    BYTE fgOverLoad[2], fgHallFail[2], fgInvVel[2], fgStall[2];

    BYTE fgEmerON, fgBatChargeON, fgRccState;

    short sCmdLinearVel, sCmdAngularVel;
    short sRealLinearVel, sRealAngularVel;

    float fRealLinearVel, fRealAngularVel;

    int nAngleResolution;

    BYTE fgIntSpeed[2], fgDirection[2], fgRunBrake[2], fgStartStop[2];
    BYTE fgEncoderA[2], fgEncoderB[2];
    BYTE byIOMonitor[2];

}MotorDriver;
MotorDriver Md;

typedef struct {
  double posx, posy, posth;
  double velx, angz;
  double pre_posx, pre_posy, pre_ori;
} freewayOdom;
freewayOdom fo;

//It is a message callback function.
//It is a function that oprates when a topic message named 'monitor_topic' is received.
//The input message is to receive the 'monitor_msg' message from 'md' package in msg directory
void monitorCallBack(const md::monitor_msg::ConstPtr& monitor)
{
    int nGap;
    static int nExsTheta;

    Md.lPosiX             = monitor->lPosiX;
    Md.lPosiY             = monitor->lPosiY;
//    Md.sTheta             = monitor->sTheta;
    Md.fTheta             = (float)(monitor->sTheta)/10;
    Md.sRealLinearVel     = monitor->sRealLinearVel;
    Md.fRealAngularVel    = (float)(monitor->sRealAngularVel)/10;
    Md.sVoltIn            = monitor->sVoltIn;
    Md.sCurrent[LEFT]     = monitor->sLeftMotCur;
    Md.sCurrent[RIGHT]    = monitor->sRightMotCur;
    Md.byUS1              = monitor->byUS1;
    Md.byUS2              = monitor->byUS2;
    Md.byUS3              = monitor->byUS3;
    Md.byUS4              = monitor->byUS4;
    Md.byPlatStatus       = monitor->byPlatStatus;
    Md.byDocStatus        = monitor->byDocStatus;
    Md.byMotStatus[LEFT]  = monitor->byLeftMotStatus;
    Md.byMotStatus[RIGHT] = monitor->byRightMotStatus;
    Md.byIOMonitor[LEFT]  = monitor->byLeftIOMonitor;
    Md.byIOMonitor[RIGHT] = monitor->byRightIOMonitor;

    Md.fgEmerON      = Md.byPlatStatus & BIT0;
    Md.fgBatChargeON = Md.byDocStatus & BIT0;
    Md.fgRccState    = (Md.byDocStatus & BIT7) >> 7;

    Md.fgAlarm[LEFT]    = Md.byMotStatus[LEFT] & BIT0;
    Md.fgCtrlFail[LEFT] = Md.byMotStatus[LEFT] & BIT1;
    Md.fgOverVolt[LEFT] = Md.byMotStatus[LEFT] & BIT2;
    Md.fgOverTemp[LEFT] = Md.byMotStatus[LEFT] & BIT3;
    Md.fgOverLoad[LEFT] = Md.byMotStatus[LEFT] & BIT4;
    Md.fgHallFail[LEFT] = Md.byMotStatus[LEFT] & BIT5;
    Md.fgInvVel[LEFT]   = Md.byMotStatus[LEFT] & BIT6;
    Md.fgStall[LEFT]    = Md.byMotStatus[LEFT] & BIT7;

    Md.fgAlarm[RIGHT]    = Md.byMotStatus[RIGHT] & BIT0;
    Md.fgCtrlFail[RIGHT] = Md.byMotStatus[RIGHT] & BIT1;
    Md.fgOverVolt[RIGHT] = Md.byMotStatus[RIGHT] & BIT2;
    Md.fgOverTemp[RIGHT] = Md.byMotStatus[RIGHT] & BIT3;
    Md.fgOverLoad[RIGHT] = Md.byMotStatus[RIGHT] & BIT4;
    Md.fgHallFail[RIGHT] = Md.byMotStatus[RIGHT] & BIT5;
    Md.fgInvVel[RIGHT]   = Md.byMotStatus[RIGHT] & BIT6;
    Md.fgStall[RIGHT]    = Md.byMotStatus[RIGHT] & BIT7;

    Md.fgIntSpeed[LEFT]  = Md.byIOMonitor[LEFT] & BIT0;
    Md.fgDirection[LEFT] = (Md.byIOMonitor[LEFT] & BIT2)>>2;
    Md.fgRunBrake[LEFT]  = (Md.byIOMonitor[LEFT] & BIT3)>>3;
    Md.fgStartStop[LEFT] = (Md.byIOMonitor[LEFT] & BIT4)>>4;
    Md.fgEncoderA[LEFT]  = (Md.byIOMonitor[LEFT] & BIT5)>>5;
    Md.fgEncoderB[LEFT]  = (Md.byIOMonitor[LEFT] & BIT6)>>6;

    Md.fgIntSpeed[RIGHT]  = Md.byIOMonitor[RIGHT] & BIT0;
    Md.fgDirection[RIGHT] = (Md.byIOMonitor[RIGHT] & BIT2)>>2;
    Md.fgRunBrake[RIGHT]  = (Md.byIOMonitor[RIGHT] & BIT3)>>3;
    Md.fgStartStop[RIGHT] = (Md.byIOMonitor[RIGHT] & BIT4)>>4;
    Md.fgEncoderA[RIGHT]  = (Md.byIOMonitor[RIGHT] & BIT5)>>5;
    Md.fgEncoderB[RIGHT]  = (Md.byIOMonitor[RIGHT] & BIT6)>>6;

    // my custom, x, y converted
    fo.posx = (double)(Md.lPosiY/1000.0); // mm -> m
    fo.posy = (double)(-Md.lPosiX/1000.0); // mm -> m
    //fo.posth = Md.fTheta ? (double)DEG2RAD(-Md.fTheta) : 0;
    //fo.posth = Md.fTheta ? (double)((float)(monitor->sTheta/10.0))*3.141592/180.0 : 0.0;
    fo.posth = (double)((float)(Md.fTheta))*Pi/180.0;
    fo.velx = (double)Md.sRealLinearVel/1000.0; // mm/s -> m/s
    fo.angz = (double)(Md.fRealAngularVel*Pi/180.0); // 10deg/sec -> rad/sec * 10


    //nGap = Md.sTheta - nExsTheta;
    //ROS_INFO("%4d %4d", nGap, Md.sTheta);
    //nExsTheta = Md.sTheta;
    //ROS_INFO("%d, %d, %d, %d", Md.sCmdLinearVel, Md.sRealLinearVel, Md.sCmdAngularVel, Md.sRealAngularVel);
    if(debug_print) printf("sub-> x: %lf  y: %lf  theta(DEG): %lf theta(RAD): %lf  linearVel: %f  angularVel: %f  L_Cur: %d  R_Cur: %d  US1:%d  US2:%d  US3:%d  "
           "volt:%d  Emergecy:%d  Charge:%d  DocState:%d  LMotStatu:%d  RMotStatu:%d  LDirPin:%d  LSSPin:%d  "
           "RDirPin:%d  RSSPin:%d \n",
          fo.posx, fo.posy, Md.fTheta, fo.posth,fo.velx, fo.angz, Md.sCurrent[LEFT], Md.sCurrent[RIGHT], Md.byUS1, Md.byUS2, Md.byUS3, Md.sVoltIn,
           Md.fgEmerON, Md.fgBatChargeON, Md.fgRccState, Md.byMotStatus[LEFT], Md.byMotStatus[RIGHT], Md.fgDirection[LEFT], Md.fgStartStop[LEFT]
           , Md.fgDirection[RIGHT], Md.fgStartStop[RIGHT]);
}
/////////////////////////////////////for example to get the 'Md.sCmdAngularVel'////////////////
void keyboardCallBack(const geometry_msgs::Twist& keyVel)  
{
    Md.sCmdLinearVel  = keyVel.linear.x * 1000;// mm/s
    if(Md.nAngleResolution)
        Md.sCmdAngularVel = keyVel.angular.z * 180/Pi * 10 ;// 0.1 deg/s -> Rad to Deg
    else
        Md.sCmdAngularVel = keyVel.angular.z * 180/Pi;// 1 deg/s -> Rad to Deg
}
///////////////////////////////////////////////////////////////////////////////////////////////
bool moving_check(double posx, double posy, double ori, double moving_check_time) {
    if(posx == fo.pre_posx && posy == fo.pre_posy && ori == fo.pre_ori) {
        // not_moving_count++;
        // if (not_moving_count >= 100) {
        //     not_moving_count = 100;
        //     return true;
        // }
        // mct = moving_check_time;
        //ros::Time::now().toSec();
        mct = mct;
    }
    else mct= moving_check_time;

    if (moving_check_time-mct > 5.0) {
        return true;
    }
    else {
        return false;
    }
    // else {
    //     not_moving_count = 0;
    //     return false; 
    // }
}

//Node main function
int main(int argc, char** argv)
{
    static int nResolution, nCnt1, nOperMode, nResetOdometry, nResetAngle, nResetAlarm;

    ros::init(argc, argv, "vel_cmd_node");                                                //Node name initialization.
    ros::NodeHandle nh;                                                                   //Node handle declaration for communication with ROS system.
    ros::Publisher cmd_vel_pub = nh.advertise<md::vel_msg>("vel_topic", 10);             //Publisher declaration.
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom_md", 50);
    ros::Publisher moving_check_pub = nh.advertise<std_msgs::Bool>("freeway/moving_check", 10);
    ros::Subscriber monitor_sub = nh.subscribe("monitor_topic", 10, monitorCallBack);    //Subscriber declaration.
    ros::Subscriber keyboard_sub = nh.subscribe("cmd_vel", 10, keyboardCallBack);
    
    ros::Time current_time;
    mct=ros::Time::now().toSec();
    ros::Rate r(20);                                                                      //Set the loop period -> 50ms.

    nOperMode = 0;
    md::vel_msg vel;          //vel_msg declares message 'vel' as message file.
    geometry_msgs::TransformStamped odom_tf;
   // sensor_msgs::Imu imu;
    tf::TransformBroadcaster odom_broadcaster;
    tf::Transform transform;
    nav_msgs::Odometry odom;
    std_msgs::Bool moving_check_flag;

    nh.getParam("md_node/angleresolution", Md.nAngleResolution);

    while(ros::ok())
    {

        if(nCnt1++ == 10)     //Store the value of the parameter in the variable once per 500mS.
        {
            nCnt1 = 0;
            nh.getParam("vel_cmd_node/reset_odometry", nResetOdometry);
            nh.getParam("vel_cmd_node/reset_angle", nResetAngle);
            nh.getParam("vel_cmd_node/reset_alarm", nResetAlarm);
        }

        vel.nLinear         = Md.sCmdLinearVel;
        vel.nAngular        = Md.sCmdAngularVel;
        vel.byResetOdometry = nResetOdometry;
        vel.byResetAngle    = nResetAngle;
        vel.byResetAlarm    = nResetAlarm;
        cmd_vel_pub.publish(vel);                 //Publish the message 'vel'

        ros::spinOnce();

        current_time = ros::Time::now();
        double moving_check_time = ros::Time::now().toSec();

        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw((fo.posth)); //fo.posth
       // transform.setOrigin(tf::Vector3(fo.posx,fo.posy,0));
       // transform.setRotation(tf::Quaternion(quaternion.x,quaternion.y,quaternion.z,quaternion.w));
       // odom_broadcaster.sendTransform(tf::StampedTransform(transform,  current_time, "odom_md_frame", "base_footprint")); //current_time = ros::Time::now()
       
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom_md_frame";
        odom.pose.pose.position.x = fo.posx;
        odom.pose.pose.position.y = fo.posy;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = quaternion;

        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = fo.velx;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = fo.angz;

        odom.pose.covariance[0] = 0.3;
	odom.pose.covariance[7] = 0.3;
	odom.pose.covariance[14] = 9999.0;
	odom.pose.covariance[21] = 9999.0;
	odom.pose.covariance[28] = 9999.0;

        if(moving_check(fo.posx, fo.posy, quaternion.z, moving_check_time)) {
	    odom.pose.covariance[35] = 0.0;
            moving_check_flag.data = true; // when true it's not moving
        }

        else {
            moving_check_flag.data = false; // when false is's moving
	    odom.pose.covariance[35] = 1.0;
        }

        odom.twist.covariance = odom.pose.covariance;

        // odom_tf.header = odom.header;
        // odom_tf.child_frame_id = odom.child_frame_id;
        // odom_tf.transform.translation.x = odom.pose.pose.position.x;
        // odom_tf.transform.translation.y = odom.pose.pose.position.y;
        // odom_tf.transform.translation.z = odom.pose.pose.position.z;
        // odom_tf.transform.rotation = odom.pose.pose.orientation;
        // odom_broadcaster.sendTransform(odom_tf);
        
        moving_check_pub.publish(moving_check_flag);                 
        odom_pub.publish(odom);
        fo.pre_posx = fo.posx;
        fo.pre_posy = fo.posy;
        fo.pre_ori = quaternion.z;
        // ros::spinOnce();
        r.sleep();                                //Go to sleep according to the loop period defined
    }
}
