#ifndef  _CORRELATIVE_MATCHES_H_
#define _CORRELATIVE_MATCHES_H_

#include<assert.h>
#include<vector>
#include<cstdlib>

#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<geometry_msgs/Transform.h>
#include<tf/transform_listener.h>
#include<tf/tf.h>

#include<eigen3/Eigen/Dense>

//#include<csm/csm.h>

#include<cmath>
#include <boost/graph/graph_concepts.hpp>
#include "predeal.hpp"


using namespace std;




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
  
 void setParams(const float& Windowsizes_,
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

  /**
   * \brief generate a guassian kernel matrix ,which is used to smear the likelihoodFiled
   * \param[in] sizes the size of guassian kernel matrix 
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
     int x=centriod_gridType(0)-size_girdType/2;
     int y=centriod_gridType(1)-size_girdType/2;
     
     //likehoodFiledMap.block(x,y,size_girdType,size_girdType)+=guassK;
     /*
     for(int i=centriod_gridType(0)-size_girdType/2,m=0;i<centriod_gridType(0)+size_girdType/2;i++,m++)
      {
	for(int j=centriod_gridType(1)-size_girdType/2,n=0;j<=centriod_gridType(1)+size_girdType/2;j++,n++)
	{
	    if(m>=Windowsizes||n>=Windowsizes)
	    {
	      cout<<"矩阵索引超出边界"<<endl;
	      assert(m<Windowsizes&&n<Windowsizes);
	    }
	   likehoodFiledMap(i,j)+=guassK(m,n);
	   if(likehoodFiledMap(i,j)>10*getMaxCoffsGuassianK())
	   {
	     likehoodFiledMap(i,j)=10*getMaxCoffsGuassianK();
	  }	  
	}
      }
      */
      
      
      
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
  
  
  /**
   * 初始化为零，重新创建
   */
    Eigen::MatrixXf create(const sensor_msgs::PointCloud &mapPC)
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
  
  inline float getMaxCoffsGuassianK() const
  {
    return guassK.maxCoeff();
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
  CorrelativeMatch()  {
    searchMethod="brute force";
    favor_threshold=1.0;
  }
  
  CorrelativeMatch( Eigen::Vector3f poseWindowSizes_)
  {
    poseWindowSizes=poseWindowSizes_;
  }
  
  /*
   * 复制构造函数
   * TODO
   * 
   */
  void setSearchWindowSizes(Eigen::Vector3f poseWindowSizes_)
  {
    poseWindowSizes=poseWindowSizes_;
  }
  
  void setSearchStepLength(const float &x_resolution_, const float &y_resolution_,const float &orientation_resolution_)
  {
    assert(x_resolution_>0&&y_resolution_>0&&orientation_resolution_>0);
    x_resolution=x_resolution_;
    y_resolution=y_resolution_;
    orientation_resolution=orientation_resolution_;
  }
  
  void setSearchMethod(const string &searchMethod_)
  {
    searchMethod=searchMethod_;
  }
  
  void setFavorThreshod(const float &percent)
  {
    favor_threshold=percent;
  }
 
 
 /**
  * 使用暴力查找的方法
  */  
  Eigen::Vector3f BruteForceLookup() const
  {
    float bound=pc_base.points.size()*favor_threshold*llf.getMaxCoffsGuassianK();
    float score_Max=score(poseWindowCentriod);
    if(score_Max>bound)
    {
      return poseWindowCentriod;
    }
    Eigen::Vector3f beset_state=poseWindowCentriod;
    for(int i=0;i<floor(poseWindowSizes(0)/x_resolution);i++) 
    {
       float pose_x=poseWindowCentriod(0)-poseWindowSizes(0)/2+i*x_resolution;
      for(int j=0;j<floor(poseWindowSizes(1)/y_resolution);j++)
      {
	  float pose_y=poseWindowCentriod(1)-poseWindowSizes(1)/2+j*y_resolution;
	for(int k=0;k<floor(poseWindowSizes(2)/orientation_resolution);k++)
	{	
	  float theta=poseWindowCentriod(2)-poseWindowSizes(2)/2+k*orientation_resolution;
	  /*
	  tf::Transform tf_;
	  tf::Quaternion q(0,0,sin(theta/2),cos(theta/2));
	  tf::Vector3 Origin(pose_x,pose_y,0);	  
	  tf_.setRotation(q);
	  tf_.setOrigin(Origin);
	  */
	  Eigen:: Vector3f state(pose_x,pose_y,theta);
	  float scoreValue=score(state);
	  if(scoreValue>score_Max&&scoreValue<=bound)
	  {
	    cout<<"发现一个更好的pose"<<endl;
	    score_Max=scoreValue;
	    beset_state=state;
	  }
	  else if (scoreValue>bound)
	  {
	    return state;
	  }
	}
      }
    }
    return beset_state;
    //return poseWindowCentriod;
  }
  
  
  
  //分片的目的是为了节约运行时间，如我们在旋转这一层先计算旋转，那么内部就可以少计算N*N次旋转，  
   Eigen::Vector3f SilceLookup() const
  {
    float bound=pc_base.points.size()*favor_threshold*llf.getMaxCoffsGuassianK();
    float score_Max=score(poseWindowCentriod);
    if(score_Max>=bound)
    {
      return poseWindowCentriod;
    }
    
    int theta_sizes=floor(poseWindowSizes(2)/orientation_resolution);
    int x_sizes=floor(poseWindowSizes(0)/x_resolution);
    int y_sizes=floor(poseWindowSizes(1)/y_resolution);
    
    
    Eigen::Vector3f best_state=poseWindowCentriod;
 
    
    
    
    for(int i=0;i<theta_sizes;i++) 
    {
       sensor_msgs::PointCloud TransformedPC;
      float theta=poseWindowCentriod(2)-poseWindowSizes(2)/2+i*orientation_resolution;
      Eigen::Quaternion<float> q(cos(theta)/2,0,0,sin(theta)/2);
       for(int i_=0;i_<pc_base.points.size();i_++)
	{	
	    Eigen::Vector3f tempoint;
	    tempoint<<pc_base.points[i_].x, pc_base.points[i_].y,0;
	    tempoint=q.toRotationMatrix()*tempoint;
	     geometry_msgs::Point32 tempointx;
	  tempointx.x=tempoint(0);tempointx.y=tempoint(1);tempointx.z=0;
	  TransformedPC.points.push_back(tempointx);
	} 
      
      for(int j=0;j<x_sizes;j++)
      {
	float pose_x=poseWindowCentriod(0)-poseWindowSizes(0)/2+j*x_resolution;
	  
	for(int k=0;k<y_sizes;k++)
	{	  
	  float pose_y=poseWindowCentriod(1)-poseWindowSizes(1)/2+k*y_resolution;
	  float sum=1;
	 for(int ii=0;ii<TransformedPC.points.size();ii++)
	{   
	  Eigen::Vector2i temIndex;
	  Eigen::Vector2f point2d;
	  point2d(0)=TransformedPC.points[ii].x+pose_x;
	  point2d(1)=TransformedPC.points[ii].y+pose_y;	  
	   temIndex=preDeal::world2grid(llf.getMapResolution(),llf.getMapOrigin(),point2d);
	  if(temIndex(0)>=0&&temIndex(0)<llfmap.rows() &&temIndex(1)>=0&&temIndex(1)<llfmap.cols())
	  sum*=llfmap(temIndex(0),temIndex(1));	    
	} 
	  Eigen:: Vector3f state(pose_x,pose_y,theta);
	  if(sum>score_Max&&sum<=bound)
	  {
	    cout<<"发现一个更好的pose"<<endl;
	    score_Max=sum;
	    best_state=state;
	  }
	  else if (sum>bound)
	  {
	    return state;
	  }
	}
      }
    }
    return best_state;
  }
  
  
  
  
  
  //在点云常用的缩写是pc
  //需要验证，ros中所有的stampTransform代表的意思都是得到转换关系都是把child_frame_id数据处理到frame_id下，而且设置transform时，也是必须按照这个设置
  
  /**
   * \brief 对转换关系进行打分，分值等于所有的激光hits的点被转换到map坐标系下，在似然场中的分数的乘积，比用求和的效果要好
   * \param[in] tf_ laser到map的转换关系
   */
  float score(const tf::Transform &tf_) const
  {
    float tf_score=1;        
   for(int i=0;i<pc_base.points.size();i++)
   {
     Eigen::Vector2i temIndex;
     Eigen::Vector3f tempoint;
     Eigen::Vector2f point2d;
     tempoint<<pc_base.points[i].x, pc_base.points[i].y,0;
     Eigen::Quaternion<float> q(tf_.getRotation().w(),tf_.getRotation().x(),tf_.getRotation().y(),tf_.getRotation().z());
     Eigen::Vector3f trans(tf_.getOrigin().x(),tf_.getOrigin().y(),tf_.getOrigin().z());
     tempoint=q.toRotationMatrix()*tempoint+trans;
     point2d<<tempoint(0),tempoint(1);
     temIndex=preDeal::world2grid(llf.getMapResolution(),llf.getMapOrigin(),point2d);
     if(temIndex(0)>=0&&temIndex(0)<llfmap.rows() &&temIndex(1)>=0&&temIndex(1)<llfmap.cols())
     tf_score*=llfmap(temIndex(0),temIndex(1));     
  }    
  }
  
  float score(const Eigen::Vector3f &tf_) const
  {
    float tf_score=0;        
   for(int i=0;i<pc_base.points.size();i++)
   {
     assert(!pc_base.points.empty());
     Eigen::Vector2i temIndex;
     Eigen::Vector3f tempoint;
     Eigen::Vector2f point2d;
     tempoint<<pc_base.points[i].x, pc_base.points[i].y,0;
     Eigen::Quaternion<float> q(cos(tf_(2)/2),0,0,sin(tf_(2)/2));
     Eigen::Vector3f trans(tf_(0),tf_(1),0);
     tempoint=q.toRotationMatrix()*tempoint+trans;
     point2d<<tempoint(0),tempoint(1);
     temIndex=preDeal::world2grid(llf.getMapResolution(),llf.getMapOrigin(),point2d);
     if(temIndex(0)>=0&&temIndex(0)<llfmap.rows() &&temIndex(1)>=0&&temIndex(1)<llfmap.cols())
     tf_score+=llfmap(temIndex(0),temIndex(1));     
  }
  return tf_score;
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
  
  /*错误的做法
  Eigen::Vector3f getCorrelativePose(const sensor_msgs::LaserScan & scan_,const Eigen::Vector3f &poseWindowCentriod_)
  {
    preDeal::scan2pc(scan_,currentPCinLaserFrame);
    poseWindowCentriod=poseWindowCentriod_;
    relativePose=lookup();
    return relativePose;
  }
  */
  
    Eigen::Vector3f getCorrelativePose(const sensor_msgs::PointCloud & pc_base_,const Eigen::Vector3f &poseWindowCentriod_)
  {
    pc_base=pc_base_;
    poseWindowCentriod=poseWindowCentriod_;
   if(searchMethod=="brute force" )
   {
     relativePose=BruteForceLookup();
  }
  else if(searchMethod=="slice")
  {
     relativePose=SilceLookup();
  }
  else
  {
    cout<<"没有相应的搜索方法可以匹配，请重新输入 "<<endl;
    exit(0);
  }
    // relativePose=lookup();
   
    return relativePose;
  }
  
private:
  //搜索时x的分辨率
  float x_resolution;
  //搜索时的分辨率
  float y_resolution;
  //搜索时角度的分辨虚
  float orientation_resolution;
  //窗的位置，由中心决定
  Eigen::Vector3f poseWindowCentriod;
  //窗的尺寸，单位是米，认为传感器的误差越大，则窗应该设置的越大
  Eigen::Vector3f poseWindowSizes;
  //目前不是相对的，得到的直接就是位姿
  Eigen::Vector3f relativePose;
  //在base坐标系下的点云数据
  sensor_msgs::PointCloud pc_base;
  //似然场
 likelihoodFiled llf;
 //存放似然场的地图信息
 Eigen::MatrixXf llfmap;
 //搜索位姿的方法
 string searchMethod;
 //阈值
 float favor_threshold;
};


}


#endif
