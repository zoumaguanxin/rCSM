#ifndef _CSM_ROS_
#define _CSM_ROS_
#include "CorrelativeMatch.hpp"
#include <boost/graph/graph_concepts.hpp>
#include <tf2_ros/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf2_ros/buffer.h>
#include<octomap_msgs/Octomap.h>

namespace CSM_ROS{

class roscsm{
public:
roscsm(ros::NodeHandle &n_):tf_listener(n_,ros::DURATION_MAX)
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
 V_sub.push_back(subscan);
  nh.param<float>("windowsize",windowsize,0.5f);
  nh.param<int>("localmapsize",localmapsize,800);
  nh.param<float>("map_resolution",map_resolution,0.05);
  Eigen::Vector3f poseWindowsize_;
  poseWindowsize_.setOnes();
  poseWindowsize_(2)=3.14*2/3.f;
  poseWindowsize=poseWindowsize_;
  //nh.param<Eigen::Vector3f>("poseWindowsize",poseWindowsize,poseWindowsize_);
  nh.param<float>("sigma",sigma,0.1);
}




bool scan2foopc( const sensor_msgs::LaserScan& curScan,const string &target_frame, sensor_msgs::PointCloud& pcout)
{
  
   sensor_msgs::PointCloud pcin;
  for(int i=0;i<curScan.ranges.size();i++)
  {
    geometry_msgs::Point32 point;
    if(current_scan.ranges[i]<current_scan.range_max)
    {
    point.x=curScan.ranges[i]*cos(curScan.angle_min+i*curScan.angle_increment);
    point.y=curScan.ranges[i]*sin(curScan.angle_min+i*curScan.angle_increment);
    point.z=0;
    pcin.points.push_back(point);
    }
  }
  pcin.header.frame_id=curScan.header.frame_id;
  cout<<"scan frame:"<<current_scan.header.frame_id<<endl;
  pcin.header.stamp=current_scan.header.stamp;
  cout<<"this is executed"<<endl;
  
  bool transflag=false;
  geometry_msgs::TransformStamped tfstamp;
 // if(tf_listener.canTransform(target_frame,current_scan.header.frame_id,ros::Time(0)))
  //{
 
try{
  tf_listener.transformPointCloud(target_frame,pcin,pcout);
  transflag=true;
  cout<<"transform is ok"<<endl;
}
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();  
  }
//   }
//   else{
//     return transflag;
//   }
  return transflag; 
}


void run()
{  
  
//   bool getpose=false;
//   tf::StampedTransform laser2odom;
//   // geometry_msgs::TransformStamped laser2odom;
//   //  tf2_ros::Buffer tfBuffer;
//    //tf2_ros::TransformListener tf_listener(tfBuffer);
//    while(!getpose)
//    {
//    try{
//     //laser2odom= tfBuffer.lookupTransform("odom","laser",ros::Time(0));
//      double t=10000.0;
//       tf_listener.waitForTransform("/odom","/laser",ros::Time(0),ros::Duration(t));
//       tf_listener.lookupTransform("/odom","/laser",ros::Time(0),laser2odom);
//    getpose=true;
//     }
//     catch(tf2::LookupException &ex)
//     {
//       ROS_ERROR("%s",ex.what());
//       ros::Duration(1.0).sleep();  
//     }
//    }
  ros::Rate r(50);
 while(ros::ok())
 {
    ros::spinOnce();
   if(current_scan.ranges.size()>0)
   {     
/*
    Eigen::Vector3f poseWindowCentriod;
    poseWindowCentriod<<laser2odom.getOrigin().getX(),laser2odom.getOrigin().getY(),2*acos(laser2odom.getRotation().getW());*/
    //创建似然场
    //cout<<windowsize<<endl;
     csm::likelihoodFiled llf(windowsize,sigma,map_resolution,localmapsize);
    sensor_msgs::PointCloud odompc;
    if(scan2foopc(current_scan,"odom",odompc))
    {
    cout<<"开始更新"<<endl;      
    Eigen::MatrixXf localmap=llf.update(odompc);
    //cout<<localmap;
    }
    r.sleep();
   }
 }
//csm::CorrelativeMatch rcsm(current_scan,poseWindowCentriod,poseWindowsize,scan2pointCloud,windowsize,sigma,map_resolution);
//rcsm.setParam(0.05,0.05,1);
}


  
private:
ros::NodeHandle nh;
sensor_msgs::LaserScan current_scan;
float windowsize;
int localmapsize;
tf::TransformListener tf_listener;
float map_resolution;
float sigma;
Eigen::Vector3f poseWindowsize;
//订阅消息，必须具有静态生命周期，所以选择写为类的私有成员，在主程序中由于类的对象具有静态生命周期
//所以该订阅者都具有静态生命周期，保证了在程序运行期间，该订阅者一直存在
ros::V_Subscriber V_sub;
};

}


#endif