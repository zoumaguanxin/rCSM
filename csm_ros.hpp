#ifndef _CSM_ROS_
#define _CSM_ROS_
#include "CorrelativeMatch.hpp"
#include <boost/graph/graph_concepts.hpp>
#include <tf2_ros/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf2_ros/buffer.h>
#include<octomap_msgs/Octomap.h>
#include<nav_msgs/OccupancyGrid.h>
#include<ctime>
#include <queue>


#define pi 3.1415926

void operator+=(std::queue<sensor_msgs::PointCloud> &pcqueue, const sensor_msgs::PointCloud& pc_right)
{
  if(pcqueue.size()<10)
  {
    pcqueue.push(pc_right);
  }
  else{
    pcqueue.pop();
    pcqueue.push(pc_right);
  }
}

void operator+=(sensor_msgs::PointCloud & pc_left,const sensor_msgs::PointCloud &pc_right)
{
  sensor_msgs::PointCloud temppc(pc_left);
 cout<<"temppc max size "<<temppc.points.max_size()<<endl;
  cout<<"pc_right size "<<pc_right.points.size()<<endl;
  for(int i=0;i<pc_right.points.size();i++)
  {
    pc_left.points.push_back(pc_right.points[i]);
  }
 pc_left.header.stamp=pc_right.header.stamp;
 cout<<"temppc size: "<<pc_left.points.size()<<endl;
 //return  temppc;
}

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
 * \param poseWindowsize_theta default pi/3.f, unit rad.
 * \param pose_x_resolution default 0.03
 * \param pose_y_resolution default 0.03
 * \param pose_theta_resolution default 1/180*3.14*2 
 */
void init()
{
  ros::Subscriber subscan=nh.subscribe<sensor_msgs::LaserScan>("scan",10,boost::bind(&roscsm::callbackScan, this,_1));
 V_sub.push_back(subscan);
  nh.param<int>("windowsize",windowsize,9);
  nh.param<int>("localmapsize",localmapsize,800);
  nh.param<float>("map_resolution",map_resolution,0.05);
  nh.param<float>("sigma",sigma,0.1);
  nh.param<float>("poseWindowsize_x",poseWindowsize(0),0.5);
  nh.param<float>("poseWindowsize_y",poseWindowsize(1),0.5);
  nh.param<float>("poseWindowsize_theta",poseWindowsize(2),pi/2.f);
  nh.param<string>("searchMethod",searchMethod,"slice");
  float favor_threshod;
  nh.param<float>("favor_threhod",favor_threshod,1.0);
  float pose_x_resolution, pose_y_resolution, pose_theta_resolution;
  nh.param<float>("pose_x_resolution",pose_x_resolution,0.05);
  nh.param<float>("pose_x_resolution",pose_y_resolution,0.05);
  nh.param<float>("pose_x_resolution",pose_theta_resolution,1/180.f*3.14*2);
  nh.param<string>("odom_frame",odom_frame_,"odom");
  nh.param<string>("laser_frame",laser_frame_,"laser");
  nh.param<string>("base_frame",base_frame_,"base_link");
  pubGridMap=nh.advertise<nav_msgs::OccupancyGrid>("gridMap",1);
  pubpc=nh.advertise<sensor_msgs::PointCloud>("odompc",1);
  pubfixedPC=nh.advertise<sensor_msgs::PointCloud>("fixedPC",1);
   //csm::likelihoodFiled llf_(windowsize,sigma,map_resolution,localmapsize);
   //csm::CorrelativeMatch rcsm;
   rcsm.setSearchWindowSizes(poseWindowsize);
   rcsm.setSearchStepLength(pose_x_resolution,pose_y_resolution,pose_theta_resolution);
   rcsm.setSearchMethod(searchMethod);
   rcsm.setFavorThreshod(favor_threshod);
   llf.setParams(windowsize,sigma,map_resolution,localmapsize);
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
	   // cout<<"transform is ok"<<endl;
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

void publishGridMap(const Eigen::MatrixXf & localmap_)
{
     //**************************************************************
    //用栅格地图表示出似然场
    //**************************************************************
    
    // localmap=llf.update(odompc);//用来测试各种转换是否正常
    //除以最大值，方便转换为OccupancyGrid表示
  Eigen::MatrixXf localmap=localmap_/localmap_.maxCoeff();
    
    //标记一下矩阵原点附近
    Eigen::MatrixXf gaussK;
   gaussK= llf.generateGuassKernal(9);
   localmap.block(10,10,9,9)=gaussK;
 
   //用栅格地图表示出似然场
    nav_msgs::OccupancyGrid OG;
    OG.header.stamp=ros::Time::now();
    OG.header.frame_id=odom_frame_;
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


void run()
{  

  ros::Rate r(50);
  sensor_msgs::PointCloud fixedpc_all;
 while(ros::ok())
 { 
    current_scan.ranges.clear();
    ros::spinOnce();
    //是有问题的，上次的激光数据没有清零
   if(!current_scan.ranges.empty())
   {     

    sensor_msgs::PointCloud odompc,fixedpc, pc_base; 
    Eigen::MatrixXf localmap;
    if(scan2foopc(current_scan,odom_frame_,odompc)&&scan2foopc(current_scan,base_frame_,pc_base))
    {
      //发布用odom转换得到的点云。
     pubpc.publish(odompc);
     cout<<"成功处理为点云"<<endl;
     
     //*****************************************************
     //csm的主要程序
     //******************************************************
     time_t start, finish;
     start=clock();
     if(!rcsm.llfIsEmpty())
     {
       Eigen::Vector3f poseWindowCentriod;
       if(getOdomPose(odom_frame_,base_frame_,ros::Time(0),poseWindowCentriod))
       {
	 Eigen::Vector3f updatedPose;
         updatedPose=rcsm.getCorrelativePose(pc_base,poseWindowCentriod);
	 preDeal::transformPointCloud(pc_base,fixedpc,updatedPose,odom_frame_);
	 if(fixedpc_all.points.empty())
	 {
	   fixedpc_all=fixedpc;
	   }
	 else{	   
	  fixedpc_all+=fixedpc;
	 }
	 //preDeal::transformPointCloud(pc_base,fixedpc,poseWindowCentriod,"odom"); //测试订阅里程计数据处理点云是否正确
	 pubfixedPC.publish(fixedpc_all);
	 cout<<"开始更新似然场"<<endl; 
	 localmap=llf.update(fixedpc_all);
       }
    }
    else
    {
       cout<<"开始更新似然场"<<endl; 
      localmap=llf.update(odompc);
    }
    finish=clock();
    cout<<"运行时间："<<(double)(finish-start)/CLOCKS_PER_SEC;
    //更新rcsm中的似然场
    rcsm.updataikehoodField(llf);
    /**********************************************************/
    
    
    //**************************************************************
    //用栅格地图表示出似然场
    //**************************************************************
    
    // localmap=llf.update(odompc);//用来测试各种转换是否正常
    //除以最大值，方便转换为OccupancyGrid表示
    publishGridMap(localmap);
    }
    r.sleep();
   }
 }
}

  
private:
  //坐标系定义
 string odom_frame_, laser_frame_, base_frame_;
ros::NodeHandle nh;
sensor_msgs::LaserScan current_scan;
//高斯核矩阵的大小
int windowsize;
//二维高斯分布中的标准差
float sigma;
//地图的大小
int localmapsize;
//搜索方法
string searchMethod;
//rcsm, 注意没有生成协方差
csm::CorrelativeMatch rcsm;
//似然场
csm::likelihoodFiled llf;
//关键，注意这个是可以作为参数传递给其他函数的，相当于它自身里有一个容器，它存储了一段时间内tf变换
tf::TransformListener tf_listener;
//地图分辨率
float map_resolution;
//位姿的搜索空间大小
Eigen::Vector3f poseWindowsize;
//订阅消息，必须具有静态生命周期，所以选择写为类的私有成员，在主程序中由于类的对象具有静态生命周期
//所以该订阅者都具有静态生命周期，保证了在程序运行期间，该订阅者一直存在
ros::V_Subscriber V_sub;
//发布着
ros::Publisher pubGridMap,pubpc, pubfixedPC;
};

}


#endif