#ifndef  _CORRELATIVE_MATCHES_H_
#define _CORRELATIVE_MATCHES_H_

#include<assert.h>
#include<vector>

#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Transform.h>
#include<tf/transform_listener.h>
#include<tf/tf.h>

#include<eigen3/Eigen/Dense>

//#include<csm/csm.h>

#include<cmath>

using namespace std;

namespace preDeal
{
  Eigen::Vector2f gird2world(const float &map_resolution, Eigen::Vector2i map_origin , const Eigen::Vector2i& position_grid)
  {
    Eigen::Vector2f pos;
    //x_in_map_frame=x_img_frame-map_origin_x_in_img_frame
    //y_in_map_frame=map_origin_y_in_img_frame-y_in_img_frame
    pos<<map_resolution*(map_origin(0)-position_grid(0)), map_resolution*(position_grid(1)-map_origin(1));
    return pos;
  }
  
 Eigen::Vector2i world2grid(const float &map_resolution, Eigen::Vector2i map_origin , Eigen::Vector2f& position)
 {
   Eigen::Vector2i x;
   x<<map_origin(0)-floor(position(0)/map_resolution),floor(position(1)/map_resolution)+map_origin(1);
   return x;   
}
}


namespace csm{

class likelihoodFiled{
public:
    float map_resolution;
  
  explicit likelihoodFiled(const float& Windowsizes_,
		  const float  &sigma_ ,const float &map_resolution_ , const int &mapSizes_)
  {

    Windowsizes=Windowsizes_;
    mapSizes=mapSizes_;
    sigma=sigma_;
    map_resolution=map_resolution_;
    Eigen::MatrixXf map_(mapSizes_,mapSizes_);
   likehoodFiledMap=map_.setZero();
     map_origin<<int(mapSizes/2), int(mapSizes/2);
  }
  
 float preScore(const Eigen::Vector2i& point)
 {
   float score_value;
   Eigen::Vector2f temp;
   temp=centriod-preDeal::gird2world(map_resolution,map_origin,point);   
   cout<<temp<<endl;
   score_value=1/(sqrt(2*3.141692f)*sigma)*exp(-temp.squaredNorm()/(2*pow(sigma,2)));
   return score_value;
}

Eigen::MatrixXf generateGuassKernal(const int &sizes)
{
  if((sizes%2)!=1)
  {
    cout<<"sizes必须是一个奇数"<<endl;
    assert((sizes%2)==1);
  }
  Eigen::MatrixXf K;
  K.resize(sizes,sizes);
  for(int i=0;i<(sizes/2+1);i++)
  {
    for(int j=i;j<(sizes/2+1);j++)
    {
      Eigen::Vector2f temp;
      temp<<(sizes/2-i)*map_resolution,(sizes/2-j)*map_resolution;
      K(i,j)=1/(sqrt(2*3.141692f)*sigma)*exp(-temp.squaredNorm()/(2*pow(sigma,2)));
      K(sizes-i-1,j)=K(i,j);
      if(i!=j)
      {
      K(j,i)=K(i,j);
      K(sizes-j-1,i)=K(j,i);
      }
      if((sizes-i-1)!=i)
      {
      K.block(sizes-i-1,0,1,sizes)=K.block(i,0,1,sizes);
      }
    }
  }  
}
 
 /**
  * \brief 对每一个点都创建一个似然场区域
  */
 bool  smear()
  {
    cout<<"smear begin"<<endl;
    assert(point_cloud.points.size()>0); 
    //直接去生成一个高斯核矩阵，而不再这样转换过去在转换回来
    for(int i=0;i<point_cloud.points.size();i++)
    {      
      centriod<<point_cloud.points[i].x,point_cloud.points[i].y;
      Eigen::Vector2i centriod_gridType=preDeal::world2grid(map_resolution,map_origin,centriod);
      int size_girdType=Windowsizes/map_resolution;
      cout<<"smear size:"<<size_girdType<<endl;
      cout<<Windowsizes<<endl;
      assert(size_girdType>0);
      assert((centriod_gridType(0)-size_girdType/2)>0);
      for(int i=centriod_gridType(0)-size_girdType/2;i<centriod_gridType(0)+size_girdType/2;i++)
      {
	for(int j=centriod_gridType(0)-size_girdType/2;j<=centriod_gridType(0)+size_girdType/2;j++)
	{
	  //i本身就表示y坐标方向上, j表示x坐标方向上
	  Eigen::Vector2i point(i,j);
	  cout<<preScore(point)<<endl;
	  if(preScore(point)>likehoodFiledMap(i,j))
	  {
	    //因为矩阵的行表示的y,列表示的是x
	     likehoodFiledMap(i,j)=255*preScore(point);
	     cout<<"score:"<<i<<" ,"<<j<<" "<<"score:"<<likehoodFiledMap(i,j)<<endl;
	  }
	}
      }
    }
      return true;
  }
  
  Eigen::MatrixXf getlikelihoodField()
  {
    if(smear())
    return likehoodFiledMap;
  }
  
  Eigen::MatrixXf update(const sensor_msgs::PointCloud &laser2mapPC)
  {
    assert(laser2mapPC.points.size()>0);
    point_cloud=laser2mapPC;
    if(likehoodFiledMap.rows()!=mapSizes)
    {
      likehoodFiledMap.resize(mapSizes,mapSizes);
    }
    likehoodFiledMap.setZero();
    if(smear())
   return likehoodFiledMap;
  }
  
private:
  //区域的中心
  Eigen::Vector2f centriod;
  //更新的区域大小，Windowsizes * Windowsizes的区域，为float，表示区域
  float Windowsizes;
  //用来计算概率
  float sigma;
  //存放点云，用来更新似然场
  sensor_msgs::PointCloud point_cloud;
  //似然场
  Eigen::MatrixXf likehoodFiledMap;
  int mapSizes;
  Eigen::Vector2i map_origin;
};




// class CorrelativeMatch: public likelihoodFiled{
//   
// public:
//   CorrelativeMatch(sensor_msgs::LaserScan scan_,  Eigen::Vector3f poseWindowCentriod_,  Eigen::Vector3f poseWindowSizes_,
// 		   sensor_msgs::PointCloud scan2pointCloud, int Windowsizes_, float sigma_ ,  float map_resolution_, const int &mapsize_)
//   :likelihoodFiled(scan2pointCloud,Windowsizes_,sigma_,map_resolution_,mapsize_)
//   {
//     poseWindowCentriod=poseWindowCentriod_;
//     poseWindowSizes=poseWindowCentriod_;
//     scan=scan_;
//   }
//   void setParam(const float &x_resolution_, const float &y_resolution_,const float &orientation_resolution_)
//   {
//     assert(x_resolution_>0&&y_resolution_>0&&orientation_resolution_>0);
//     x_resolution=x_resolution_;
//     y_resolution=y_resolution_;
//     orientation_resolution=orientation_resolution_;
//   }
//   
//   /**
//    * \brief 创建pose搜索窗
//    * \param[in] centriod 窗的中心
//    * \param[in] sizes_ 窗的尺寸
//    */
//   void createSearchWindow(const Eigen::Vector3f &centriod_, const Eigen::Vector3f & sizes_)
//   {
//     poseWindowSizes=sizes_;
//     poseWindowCentriod=centriod_;    
//   }
//     
//   
//   Eigen::Vector3f lookup() const
//   {
//     float score_Max=0;
//     Eigen::Vector3f state_;
//     for(int i=0;i<floor(poseWindowSizes(0)/x_resolution);i++) 
//     {
//       for(int j=0;j<floor(poseWindowSizes(1)/y_resolution);j++)
//       {
// 	for(int k=0;k<floor(poseWindowSizes(2)/orientation_resolution);i++)
// 	{
// 	  float pose_x=poseWindowCentriod(0)-poseWindowSizes(0)/2+i*x_resolution;
// 	  float pose_y=poseWindowCentriod(1)-poseWindowSizes(1)/2+i*y_resolution;
// 	  float theta=poseWindowCentriod(2)-poseWindowSizes(2)/2+i*orientation_resolution;
// 	  tf::Transform tf_;
// 	  tf::Quaternion q(0,0,sin(theta/2),cos(theta/2));
// 	  tf::Vector3 Origin(pose_x,pose_y,0);	  
// 	  tf_.setRotation(q);
// 	  tf_.setOrigin(Origin);
// 	  if(score(tf_)>score_Max)
// 	  {
// 	    score_Max=score(tf_);
// 	    Eigen::Vector3f state_(pose_x,pose_y,theta);
// 	  }	
// 	}
//       }
//     }
//     return state_;
//   }
//   
//   //在点云常用的缩写是pc
//   //需要验证，ros中所有的stampTransform代表的意思都是得到转换关系都是把child_frame_id数据处理到frame_id下，而且设置transform时，也是必须按照这个设置
//   
//   /**
//    * \brief 对转换关系进行打分，分值等于所有的激光hits的点被转换到map坐标系下，在似然场中的分数的和
//    * \param[in] tf_ laser到map的转换关系
//    */
//   float score(const tf::Transform &tf_) const
//   {
//     float tf_score=0;
//     tf::StampedTransform tf__;
//     tf__.child_frame_id_="laser";
//     tf__.frame_id_="map";
//     tf__.setOrigin(tf_.getOrigin());
//     tf__.setRotation(tf_.getRotation());
//     tf::TransformListener tf_lisenter;
//    tf_lisenter.setTransform(tf__);
//    sensor_msgs::PointCloud pcout;  
//    tf_lisenter.transformPointCloud("map",currentPCinLaserFrame,pcout);
//    for(int i=0;i<pcout.points.size();i++)
//    {
//      tf_score+=map(floor(pcout.points[i].x/map_resolution),floor(pcout.points[i].y/map_resolution));     
//   }    
//   }  
//    
//   void preComputeLikehoodField()
//   {
//     map=getlikelihoodField();  
//   }
//   
//   Eigen::Vector3f getCorrelativePose()
//   {
//     preComputeLikehoodField();
//     relativePose=lookup();
//     return relativePose;
//   }
//   
// private:
//   float x_resolution;
//   float y_resolution;
//   float orientation_resolution;
//   Eigen::Vector3f poseWindowCentriod;
//   Eigen::Vector3f poseWindowSizes;
//   Eigen::Vector3f relativePose;
//   sensor_msgs::PointCloud subMapPC;
//   sensor_msgs::LaserScan CurrentScan;
//   sensor_msgs::PointCloud currentPCinLaserFrame;
//  sensor_msgs::LaserScan scan;
//  Eigen::MatrixXf map;
// };


}


#endif
