#include "kraken_msgs/dvlData.h"
#include "kraken_msgs/depthData.h"
#include "kraken_msgs/imuData.h"
#include "resources/topicHeader.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
//#include "underwater_sensor_msgs/DVL.h"
//#include "underwater_sensor_msgs/Pressure.h"
#include "kraken_msgs/krakenPose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "pose_server/KrakenPose.h"
#include <tf/transform_datatypes.h>
const double PI=3.14;
class NavPub
{
public:
    NavPub()
    {

//        _kraken_depth=n.advertise<underwater_sensor_msgs::Pressure>(topics::SENSOR_DEPTH,10);
//        _kraken_imu=n.advertise<kraken_msgs::imuData>(topics::SENSOR_IMU,20);
//        _kraken_dvl=n.advertise<kraken_msgs::dvlData>(topics::SENSOR_DVL,10);


//        _sim_depth=n.subscribe(topics::SIMULATOR_DEPTH,10,&Converter::simDepth2krakenDepth,this);
//        _sim_dvl=n.subscribe(topics::SIMULATOR_DVL,10,&Converter::simDvl2krakenDvl,this);
//        _sim_imu=n.subscribe(topics::SIMULATOR_IMU,10,&Converter::simImu2krakenImu,this);

        _krakenpose_pub=n.advertise<kraken_msgs::krakenPose>(topics::NAV_POSE_ESTIMATED,10);
        _pose_sub=n.subscribe(topics::SIMULATOR_MODEL_POSE,10,&NavPub::pose_CB,this);
        _twist_sub=n.subscribe(topics::SIMULATOR_MODEL_TWIST,10,&NavPub::twist_CB,this);


    }
    void pose_CB(const geometry_msgs::PoseConstPtr & pose_msg)
    {
        double roll, pitch, yaw;
        _kpose.data[kraken_core::_px]=pose_msg->position.x;
        _kpose.data[kraken_core::_py]=pose_msg->position.y;
        _kpose.data[kraken_core::_pz]=pose_msg->position.z;

        tf::Quaternion q(pose_msg->orientation.x, pose_msg->orientation.y, pose_msg->orientation.z, pose_msg->orientation.w);
        tf::Matrix3x3 m(q);

		m.getRPY(roll, pitch, yaw);
		_kpose.data[kraken_core::_roll] = roll*180/PI;
        _kpose.data[kraken_core::_pitch] = pitch*180/PI;
        _kpose.data[kraken_core::_yaw] = yaw*180/PI;

        _krakenpose_pub.publish(_kpose);

    }
    void twist_CB(const geometry_msgs::TwistStampedConstPtr & twist_msg)
    {

        _kpose.data[kraken_core::_w_roll]=twist_msg->twist.angular.x;
        _kpose.data[kraken_core::_w_pitch]=twist_msg->twist.angular.y;
        _kpose.data[kraken_core::_w_yaw]=twist_msg->twist.angular.z;

        _kpose.data[kraken_core::_vx]=twist_msg->twist.linear.x;
        _kpose.data[kraken_core::_vy]=twist_msg->twist.linear.y;
        _kpose.data[kraken_core::_vz]=twist_msg->twist.linear.z;
        _krakenpose_pub.publish(_kpose);

    }



    ros::NodeHandle n;
    ros::Publisher _krakenpose_pub;
    ros::Subscriber _pose_sub,_twist_sub;
    kraken_msgs::krakenPose _kpose;
};







int main(int argc,char **argv)
{
    ros::init(argc,argv,"msg_converter");
    NavPub nv;
    ros::spin();
}
