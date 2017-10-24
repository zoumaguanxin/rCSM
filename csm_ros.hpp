#ifndef _CSM_ROS_
#define _CSM_ROS_
#include "CorrelativeMatch.hpp"
#include <boost/graph/graph_concepts.hpp>
#include <tf2_ros/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf2_ros/buffer.h>
#include<octomap_msgs/Octomap.h>
#include<nav_msgs/OccupancyGrid.h>


#define pi 3.1415926  

namespace CSM_ROS{

class roscsm{
public:
  
 /**
  * \brief the consrutor function. It is suggested that tf_listener is initialized by node and ros time although it is not neccessay to do this. 
  */
roscsm(ros::NodeHandle &n_):tf_listener(n_,ros::DURATION_MAX)
{
  nh=n_;
}

/**
 * \brief the call back function for receive the scan data
 */
void callbackScan(sensor_msgs::LaserScan::ConstPtr scanPtr)
{
  current_scan=*scanPtr;  
}


/**
 * \brief initialize some parameters include the windowsize, localmapsize, map_resolution and some publishers, Subscribers
 * \param windowsize this is a parameter that represent the size of kernel matrix used to generate the likelihoodFiled. it is a ROS parameter, if the parameter server don't set its value, it wiil be set as default 9.
 * \param sigma guass standard deviation for generating the likelihoodFiled
 *\param localmapsize this decided the sizes of localmap. also  is a ROS parameter， default 800.
 *\param map_resolution map resolution, the unit is meter. default 0.05.
 * \param poseWindowsize_x default 1.0, unit is meter.
 * \param poseWindowsize_y default 1.0 , unit meter.
 * \param poseWindowsize_theta default pi*2/3, unit rad.
 */
void init()
{
  ros::Subscriber subscan=nh.subscribe<sensor_msgs::LaserScan>("scan",10,boost::bind(&roscsm::callbackScan, this,_1));
 V_sub.push_back(subscan);
  nh.param<int>("windowsize",windowsize,9);
  nh.param<int>("localmapsize",localmapsize,800);
  nh.param<float>("map_resolution",map_resolution,0.05);
  nh.param<float>("sigma",sigma,0.1);
  nh.param<float>("poseWindowsize_x",poseWindowsize(0),0.3);
  nh.param<float>("poseWindowsize_y",poseWindowsize(1),0.3);
  nh.param<float>("poseWindowsize_theta",poseWindowsize(2),pi/3.f);
  float pose_x_resolution, pose_y_resolution, pose_theta_resolution;
  nh.param<float>("pose_x_resolution",pose_x_resolution,0.03);
  nh.param<float>("pose_x_resolution",pose_y_resolution,0.03);
  nh.param<float>("pose_x_resolution",pose_theta_resolution,1);
  pubGridMap=nh.advertise<nav_msgs::OccupancyGrid>("gridMap",1);
  pubpc=nh.advertise<sensor_msgs::PointCloud>("odompc",1);
  pubfixedPC=nh.advertise<sensor_msgs::PointCloud>("fixedPC",1);
   csm::likelihoodFiled llf_(windowsize,sigma,map_resolution,localmapsize);
   csm::CorrelativeMatch rcsm_(poseWindowsize);
   rcsm=rcsm_;
   rcsm.setParam(pose_x_resolution,pose_y_resolution,pose_theta_resolution);
   llf=llf_;
}

/**
 * \brief this function is responsible for transforming scan into point cloud data in any target frame
 *\param[in] curScan the scan data
 *\param[in] target_frame you want to transform the curScan into target frame
 *\param[out] pcout the point cloud in target frame 
 * \return if the transform has been executed successsly, the function return true, otherwise false
 */
bool scan2foopc( const sensor_msgs::LaserScan& curScan,const string &target_frame, sensor_msgs::PointCloud& pcout)
{  
  //首先转换为激光坐标系下的点云
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
      pcin.header.stamp=current_scan.header.stamp;
      
      //使用transform进行转换
      bool transflag=false;
      geometry_msgs::TransformStamped tfstamp;
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
    return transflag; 
}

/**
 * \brief 得到转换关系，如果成功则返回true， 否则返回false
 */
bool getTransform(const string &target_frame, const string &source_frame, const ros::Time&time, tf::StampedTransform &pose)
{
  bool flag=false;
  try{
    tf_listener.lookupTransform(target_frame,source_frame,time,pose);
    flag=true;
  }
  catch(tf::LookupException &ex)
  {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();  
  }
  return flag;
}

/**
 * \brief 得到base在里程计下的3个自由度的位姿，注意适用于二维的情况
 */
bool getOdomPose(const string & odom_frame,const string &base_frame,const ros::Time&time, Eigen::Vector3f &pose)
{
  tf::StampedTransform odompose;
  bool flag=false;
  try{
    tf_listener.lookupTransform(odom_frame,base_frame,time,odompose);
    flag=true;
  }
  catch(tf::LookupException &ex)
  {
	ROS_ERROR("%s",ex.what());
	ros::Duration(1.0).sleep();  
  }
  pose(0)=odompose.getOrigin().x();
  pose(1)=odompose.getOrigin().y();
  pose(2)=tf::getYaw(odompose.getRotation());
  return flag;
}


void run()
{  

  ros::Rate r(50);
 while(ros::ok())
 {
    ros::spinOnce();
    //是有问题的，上次的激光数据没有清零
   if(!current_scan.ranges.empty())
   {     

    sensor_msgs::PointCloud odompc,fixedpc, pc_base; 
    Eigen::MatrixXf localmap;
    if(scan2foopc(current_scan,"odom",odompc)&&scan2foopc(current_scan,"base_link",pc_base))
    {
      //发布用odom转换得到的点云。
     pubpc.publish(odompc);
     
     //csm
     if(!rcsm.llfIsEmpty())
     {
       Eigen::Vector3f poseWindowCentriod;
       if(getOdomPose("odom","base_link",ros::Time(0),poseWindowCentriod))
       {
	 Eigen::Vector3f updatedPose;
         updatedPose=rcsm.getCorrelativePose(pc_base,poseWindowCentriod);
	 //preDeal::transformPointCloud(pc_base,fixedpc,updatedPose,"odom");
	 preDeal::transformPointCloud(pc_base,fixedpc,poseWindowCentriod,"odom");
	 pubfixedPC.publish(fixedpc);
	 cout<<"开始更新似然场"<<endl; 
	 localmap=llf.update(fixedpc);
       }
    }
    else
    {
       cout<<"开始更新似然场"<<endl; 
      localmap=llf.update(odompc);
    }
    
    //更新rcsm中的似然场
    rcsm.updataikehoodField(llf);
    
    // localmap=llf.update(odompc);
    //除以最大值，方便转换为OccupancyGrid表示
    localmap=localmap/localmap.maxCoeff();
    
    //标记一下矩阵原点附近
    Eigen::MatrixXf gaussK;
   gaussK= llf.generateGuassKernal(9);
   localmap.block(10,10,9,9)=gaussK;
 
   //用栅格地图表示出似然场
    nav_msgs::OccupancyGrid OG;
    OG.header.stamp=ros::Time::now();
    OG.header.frame_id="odom";
    OG.info.height=localmapsize;
    OG.info.width=localmapsize;
    OG.info.resolution=map_resolution;
    
    //矩阵坐标系在地图坐标系中的位置
    geometry_msgs::Point mapPosition;
    geometry_msgs::Quaternion map_orientation;
    mapPosition.x=-localmapsize/2*0.05f;
    mapPosition.y=localmapsize/2*0.05f;
    mapPosition.z=0.0f;
    map_orientation.x=0;
    map_orientation.y=0;
    map_orientation.z=sin(-pi/4);
    map_orientation.w=cos(-pi/4);
    
    //注意，在官方文件中说是row-major order. 也就是说是行元素的内存是连续的。但是行主序去赋值，画出来的图有问题，由于Eigen存储矩阵采用的是列主序，ROS都是基于Eigen写的，所以我认为实际是列主序才对
    OG.info.origin.position=mapPosition;
    OG.info.origin.orientation=map_orientation;
    assert(localmap.rows()>0);
    for(int i=0;i<localmap.rows();i++)
    {
      for(int j=0;j<localmap.cols();j++)
      {
	if(localmap(j,i)==0)
	{
	  OG.data.push_back(-1);
	}
	else
	{
	  OG.data.push_back(100-ceil(localmap(j,i)*100));
	}
      }
    }
    pubGridMap.publish(OG);

    }
    r.sleep();
   }
 }
}


  
private:
ros::NodeHandle nh;
sensor_msgs::LaserScan current_scan;
int windowsize;
int localmapsize;
csm::CorrelativeMatch rcsm;
csm::likelihoodFiled llf;
tf::TransformListener tf_listener;
float map_resolution;
float sigma;
Eigen::Vector3f poseWindowsize;
//订阅消息，必须具有静态生命周期，所以选择写为类的私有成员，在主程序中由于类的对象具有静态生命周期
//所以该订阅者都具有静态生命周期，保证了在程序运行期间，该订阅者一直存在
ros::V_Subscriber V_sub;
ros::Publisher pubGridMap,pubpc, pubfixedPC;
};

}


#endif