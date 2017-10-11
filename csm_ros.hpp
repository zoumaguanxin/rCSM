#ifndef _CSM_ROS_
#define _CSM_ROS_
#include "CorrelativeMatch.hpp"
#include <boost/graph/graph_concepts.hpp>


class roscsm{
public:
roscsm(ros::NodeHandle &n_)
{
  nh=n_;
}

void callbackScan(sensor_msgs::LaserScan::ConstPtr scanPtr)
{
  current_scan=*scanPtr;  
}

void init()
{
  ros::Subscriber subscan=nh.subscribe<sensor_msgs::LaserScan>("scan",10,boost::bind(&roscsm::callbackScan, this,_1));
  nh.param<int>("windowsize",windowsize,9);
  nh.param<int>("localmapsize",localmapsize,800);
  nh.param<float>("map_resolution",map_resolution,0.05);
  Eigen::Vector3f poseWindowsize_=2.0f*Eigen::Vector3f::setOnes();
  poseWindowsize_(2)=3.14*2/3.f;
  nh.param<Eigen::Vector3f>("poseWindowsize",poseWindowsize,poseWindowsize_);
}

void run()
{
tf::StampedTransform laser2odom;
try{
  tf_listener.waitForTransform("odom","laser",ros::Time(0),ros::Duration(10.0));
  tf_listener.lookupTransform("odom","laser",ros::Time(0),laser2odom);
}
catch(tf::TransformException &ex)
{
  ROS_ERROR("%s",ex.what());
  ros::Duration(1.0).sleep();  
}
Eigen::Vector3f poseWindowCentriod;
poseWindowCentriod<<laser2odom.getOrigin().getX(),laser2odom.getOrigin().getY(),2*acos(laser2odom.getRotation().getW());
//创建似然场
csm::likelihoodFiled llf;
csm::CorrelativeMatch rcsm(current_scan,poseWindowCentriod,poseWindowsize,scan2pointCloud,windowsize,sigma,map_resolution);
rcsm.setParam(0.05,0.05,1);

}


  
private:
ros::NodeHandle nh;
sensor_msgs::LaserScan current_scan;
tf::TransformListener tf_listener;
int windowsize;
int localmapsize;
float map_resolution;
Eigen::Vector3f poseWindowsize;
};



#endif