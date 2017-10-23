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
#include <boost/graph/graph_concepts.hpp>


using namespace std;

namespace preDeal
{
  /**
   * \brief Transform the point in matrix frame into real world frame. The default origin of world frame is in the centriod of matrix
   */
  Eigen::Vector2f gird2world(const float &map_resolution, Eigen::Vector2i map_origin , const Eigen::Vector2i& position_grid)
  {
    Eigen::Vector2f pos;  
    //x_in_map_frame=map_origin_y_in_img_frame-y_in_img_frame
    //y_in_map_frame=x_img_frame-map_origin_x_in_img_frame
    //x_in_world=map_resolution*x_in_map_frame
    //y_in_world=map_resolution*y_in_map_frame
    pos<<map_resolution*(position_grid(1)-map_origin(1)),map_resolution*(map_origin(0)-position_grid(0));
    return pos;
  }
  
  /**
   * \brief Transform the point in real world frame  into matrix frame. 
   * \param[in] map_origin 地图原点在矩阵坐标系下的坐标
   * \param[in] map_resolution 地图的分辨率，单位是米
   * \param[in] position 地图坐标系下点的位置
   * \return 矩阵坐标系下点的位置
   */
 Eigen::Vector2i world2grid(const float &map_resolution, Eigen::Vector2i map_origin , Eigen::Vector2f& position)
 {
   Eigen::Vector2i x;
   //通过上面反推
   x<<map_origin(0)-floor(position(1)/map_resolution),floor(position(0)/map_resolution)+map_origin(1);
   return x;   
}

/**
 * \brief Transform scan into point cloud
 * \param[in] scan the scan data
 * \param[out] pcout the point cloud in scan frame
 */
void  scan2pc(const sensor_msgs::LaserScan &scan, sensor_msgs::PointCloud &pcout)
{
  if(scan.ranges.empty())
  {
    cout<<"failed to Transform scan into point cloud because the scan is empty"<<endl;
    assert(scan.ranges.size()>0);
  }
  pcout.points.reserve(scan.ranges.size());
   for(int i=0;i<scan.ranges.size();i++)
  {
    geometry_msgs::Point32 point;
    if(scan.ranges[i]<scan.range_max)
    {
    point.x=scan.ranges[i]*cos(scan.angle_min+i*scan.angle_increment);
    point.y=scan.ranges[i]*sin(scan.angle_min+i*scan.angle_increment);
    point.z=0;
    pcout.points.push_back(point);
    }
  }
      pcout.header.frame_id=scan.header.frame_id;
      pcout.header.stamp=scan.header.stamp;
}

void transformPC(const sensor_msgs::PointCloud& pcin, sensor_msgs::PointCloud &pcout, const Eigen::Vector3f &pose3d,const string & target_frame)
{
  if(!pcout.points.empty())
  {
    pcout.points.clear();    
  }
  Eigen::Quaternion<float> q(cos(pose3d(2)/2), 0,0,sin(pose3d(2)/2));
  Eigen::Vector3f trans(pose3d(0),pose3d(1),0), transformedPoint, initPoint;
  for(int i=0;i<pcin.points.size();i++)
  {
    initPoint<<pcin.points[i].x, pcin.points[i].y,0;
    geometry_msgs::Point32 tempoint;
    transformedPoint=q.toRotationMatrix()*initPoint+trans;
    transformedPoint=q.toRotationMatrix().transpose()*initPoint+trans;
    tempoint.x=transformedPoint(0); tempoint.y=transformedPoint(1);tempoint.z=0;
    pcout.points.push_back(tempoint); 
  }
     pcout.header.frame_id=target_frame;    
}

void transformPC(const sensor_msgs::LaserScan &scan, sensor_msgs::PointCloud &pcout, const Eigen::Vector3f &pose3d,const string & target_frame)
{
  sensor_msgs::PointCloud pcin;
  scan2pc(scan,pcin);
  //pcout.points.clear();
  Eigen::Quaternion<float> q(cos(pose3d(2)/2), 0,0,sin(pose3d(2)/2));
  Eigen::Vector3f trans(pose3d(0),pose3d(1),0), transformedPoint, initPoint;
  for(int i=0;i<pcin.points.size();i++)
  {
    initPoint<<pcin.points[i].x, pcin.points[i].y,0;
    geometry_msgs::Point32 tempoint;
    transformedPoint=q.toRotationMatrix()*initPoint+trans;
    tempoint.x=transformedPoint(0); tempoint.y=transformedPoint(1);tempoint.z=0;
    pcout.points.push_back(tempoint); 
  }
     pcout.header.frame_id=target_frame;    
}

}


namespace csm{

class likelihoodFiled{
public:

  likelihoodFiled(){}
  
  /**
   * \brief constructor function
   */
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
    guassK=generateGuassKernal(Windowsizes);
  }
  
  /*
   * TODO 复制构造函数
   * 
   */
  
  /*
 float preScore(const Eigen::Vector2i& point)
 {
   float score_value;
   Eigen::Vector2f temp;
   temp=centriod-preDeal::gird2world(map_resolution,map_origin,point);   
  // cout<<temp<<endl;
   score_value=1/(2*3.141692f*pow(sigma,2))*exp(-temp.squaredNorm()/(2*pow(sigma,2)));
   return score_value;
}
*/

Eigen::MatrixXf generateGuassKernal(const int &sizes)
{
  if((sizes%2)!=1)
  {
    cout<<"sizes必须是一个奇数"<<endl;
    assert((sizes%2)==1);
  }
  Eigen::MatrixXf K;
  K.resize(sizes,sizes);
  //只计算1/8的内容，其他全部复制
  for(int i=0;i<(sizes/2+1);i++)
  {
    for(int j=i;j<(sizes/2+1);j++)
    {
      Eigen::Vector2f temp;
      temp<<(sizes/2-i)*map_resolution,(sizes/2-j)*map_resolution;
      K(i,j)=1/((2*3.141692f)*pow(sigma,2))*exp(-temp.squaredNorm()/(2*pow(sigma,2)));
      //沿着以矩阵列方向中心线复制
      K(i,sizes-j-1)=K(i,j);
      if(i!=j)
      {
      //因为是对角矩阵,复制对角
      K(j,i)=K(i,j);
      K(j,sizes-i-1)=K(j,i);
      }
      if((sizes-i-1)!=i)
      {
      //以矩阵的行方向为中心线进行复制
      K.block(sizes-i-1,0,1,sizes)=K.block(i,0,1,sizes);
      }
    }
  }
  //除以最大数，因为我们希望中心是最亮的
  return K/K.maxCoeff();
}
 
 /**
  * \brief 对每一个点都创建一个似然场区域
  */
 bool  smear()
  {
    //cout<<"smear begin"<<endl;
    assert(point_cloud.points.size()>0); 
    for(int i=0;i<point_cloud.points.size();i++)
    {      
      //centriod<<point_cloud.points[i].x,point_cloud.points[i].y;
      centriod(0)=point_cloud.points[i].x;
      centriod(1)=point_cloud.points[i].y;
      Eigen::Vector2i centriod_gridType=preDeal::world2grid(map_resolution,map_origin,centriod);
      int size_girdType=Windowsizes;
      //cout<<"smear size:"<<size_girdType<<endl;
      //cout<<Windowsizes<<endl;
      assert(size_girdType>0);
      assert((centriod_gridType(0)-size_girdType/2)>0);
   
      for(int i=centriod_gridType(0)-size_girdType/2,m=0;i<centriod_gridType(0)+size_girdType/2;i++,m++)
      {
	for(int j=centriod_gridType(1)-size_girdType/2,n=0;j<=centriod_gridType(1)+size_girdType/2;j++,n++)
	{
	  if(guassK(m,n)>likehoodFiledMap(i,j))
	  {
	    if(m>=Windowsizes||n>=Windowsizes)
	    {
	      cout<<"矩阵索引超出边界"<<endl;
	      assert(m<Windowsizes&&n<Windowsizes);
	    }
	    likehoodFiledMap(i,j)=guassK(m,n);
	  }
	  /*
	   Eigen::Vector2i point(i,j);
	  if(preScore(point)>likehoodFiledMap(i,j))
	  {
	       likehoodFiledMap(i,j)=preScore(point);
	    // cout<<"score:"<<i<<" ,"<<j<<" "<<"score:"<<likehoodFiledMap(i,j)<<endl;
	  }
	  */
	}
      }      
    }    
      return true;
  }
  
  
  
  
  /**
   * \brief 是用点云更新，点云必须是已经为地图坐标系下
   */
  Eigen::MatrixXf update(const sensor_msgs::PointCloud &mapPC)
  {
    assert(mapPC.points.size()>0);
    point_cloud=mapPC;
    if(likehoodFiledMap.rows()!=mapSizes)
    {
      likehoodFiledMap.resize(mapSizes,mapSizes);
    }
    likehoodFiledMap.setZero();
    if(smear())
   return likehoodFiledMap;
  }
  
  Eigen::MatrixXf getlikelihoodField()
  {
    
    if(smear())
    return likehoodFiledMap;
  }
  
  float getMapResolution() const
  {
    return map_resolution;
  }
  
  Eigen::Vector2i getMapOrigin() const
  {
    return map_origin;
  }
  
private:

   float map_resolution;
   //区域的中心
  Eigen::Vector2f centriod;
  //更新的区域大小，Windowsizes * Windowsizes的区域，为栅格个数，必须设置为奇数，表示区域
  int Windowsizes;
  //用来计算概率
  float sigma;
  //存放点云，用来更新似然场
  sensor_msgs::PointCloud point_cloud;
  //似然场
  Eigen::MatrixXf likehoodFiledMap;
  //决定似然场的大小
  int mapSizes;
  //地图坐标系原点在矩阵坐标系下的位置，一般设置为在矩阵的中心
  Eigen::Vector2i map_origin;
  //存放高斯核，用于更新似然场
  Eigen::MatrixXf guassK;
};



class CorrelativeMatch{
  
public:
  CorrelativeMatch()  {}
  
  CorrelativeMatch( Eigen::Vector3f poseWindowSizes_)
  {
    poseWindowSizes=poseWindowSizes_;
  }
  
  /*
   * 复制构造函数
   * TODO
   * 
   */
  
  void setParam(const float &x_resolution_, const float &y_resolution_,const float &orientation_resolution_)
  {
    assert(x_resolution_>0&&y_resolution_>0&&orientation_resolution_>0);
    x_resolution=x_resolution_;
    y_resolution=y_resolution_;
    orientation_resolution=orientation_resolution_;
  }
  
 
 /**
  * TODO
  */
  
  Eigen::Vector3f lookup() const
  {
    float score_Max=0;
    Eigen::Vector3f beset_state;
    for(int i=0;i<floor(poseWindowSizes(0)/x_resolution);i++) 
    {
      for(int j=0;j<floor(poseWindowSizes(1)/y_resolution);j++)
      {
	for(int k=0;k<floor(poseWindowSizes(2)/orientation_resolution);k++)
	{
	  float pose_x=poseWindowCentriod(0)-poseWindowSizes(0)/2+i*x_resolution;
	  float pose_y=poseWindowCentriod(1)-poseWindowSizes(1)/2+j*y_resolution;
	  float theta=poseWindowCentriod(2)-poseWindowSizes(2)/2+k*orientation_resolution;
	  tf::Transform tf_;
	  tf::Quaternion q(0,0,sin(theta/2),cos(theta/2));
	  tf::Vector3 Origin(pose_x,pose_y,0);	  
	  tf_.setRotation(q);
	  tf_.setOrigin(Origin);
	  if(score(tf_)>score_Max)
	  {
	    score_Max=score(tf_);
	    Eigen::Vector3f state(pose_x,pose_y,theta);
	    beset_state=state;
	  }
	}
      }
    }
    return beset_state;
    //return poseWindowCentriod;
  }
  
  //在点云常用的缩写是pc
  //需要验证，ros中所有的stampTransform代表的意思都是得到转换关系都是把child_frame_id数据处理到frame_id下，而且设置transform时，也是必须按照这个设置
  
  /**
   * \brief 对转换关系进行打分，分值等于所有的激光hits的点被转换到map坐标系下，在似然场中的分数的和
   * \param[in] tf_ laser到map的转换关系
   */
  float score(const tf::Transform &tf_) const
  {
    float tf_score=0;        
   for(int i=0;i<currentPCinLaserFrame.points.size();i++)
   {
     Eigen::Vector2i temIndex;
     Eigen::Vector3f tempoint;
     Eigen::Vector2f point2d;
     tempoint<<currentPCinLaserFrame.points[i].x, currentPCinLaserFrame.points[i].y,0;
     Eigen::Quaternion<float> q(tf_.getRotation().w(),tf_.getRotation().x(),tf_.getRotation().y(),tf_.getRotation().z());
     Eigen::Vector3f trans(tf_.getOrigin().x(),tf_.getOrigin().y(),tf_.getOrigin().z());
     tempoint=q.toRotationMatrix()*tempoint+trans;
     point2d<<tempoint(0),tempoint(1);
     temIndex=preDeal::world2grid(llf.getMapResolution(),llf.getMapOrigin(),point2d);
     if(temIndex(0)>=0&&temIndex(0)<llfmap.rows() &&temIndex(1)>=0&&temIndex(1)<llfmap.cols())
     tf_score+=llfmap(temIndex(0),temIndex(1));     
  }    
  }  
  
  /**
   * \brief update likehood field
   * \param[in] llf_ 用于计算的似然场 
   */
  void updataikehoodField(const likelihoodFiled &llf_)
  {
    llf=llf_;
    llfmap=llf.getlikelihoodField();  
  }
  
  
  bool llfIsEmpty()
  {
    if(llfmap.cols()==0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  
  Eigen::Vector3f getCorrelativePose(const sensor_msgs::LaserScan & scan_,const Eigen::Vector3f &poseWindowCentriod_)
  {
    preDeal::scan2pc(scan_,currentPCinLaserFrame);
    poseWindowCentriod=poseWindowCentriod_;
    relativePose=lookup();
    return relativePose;
  }
  
private:
  float x_resolution;
  float y_resolution;
  float orientation_resolution;
  Eigen::Vector3f poseWindowCentriod;
  Eigen::Vector3f poseWindowSizes;
  Eigen::Vector3f relativePose;
  sensor_msgs::PointCloud currentPCinLaserFrame;
 likelihoodFiled llf;
 Eigen::MatrixXf llfmap;
};


}


#endif
