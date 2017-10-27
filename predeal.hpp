#ifndef _PREDEAL_H_
#define _PREDEAL_H_
#include<cassert>
#include<cmath>
#include<iostream>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#include<Eigen/Dense>
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


/**
 * \brief transform point cloud into target_frame using pose3d, it is noted that the transform only is suitable for 2D situation
 * \param[in] pcin the point cloud to transform
 * \param[out] pcout the point cloud transformed
 * \param[in] pose3d the pose of pcin in target_frame, it is noted that rotation represent the transform from pc_frame into target_frame
 * \param[in] target_frame it is used to label the pcout
 */
void transformPointCloud(const sensor_msgs::PointCloud& pcin, sensor_msgs::PointCloud &pcout, const Eigen::Vector3f &pose3d,const string & target_frame)
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
   // transformedPoint=q.toRotationMatrix().transpose()*initPoint+trans;
    tempoint.x=transformedPoint(0); tempoint.y=transformedPoint(1);tempoint.z=0;
    pcout.points.push_back(tempoint); 
  }
     pcout.header.frame_id=target_frame;    
}

/*
 * 有问题，不能这样做，因为laser frame到odom frame不仅仅只有俯仰角的变化，所以不能简单的用pose3d来代表transform
void transformPC(const sensor_msgs::LaserScan &scan, sensor_msgs::PointCloud &pcout, const Eigen::Vector3f &pose3d,const string & target_frame)
{
  sensor_msgs::PointCloud pcin;
  scan2pc(scan,pcin);
  pcout.points.clear();
  Eigen::Quaternion<float> q(cos(pose3d(2)/2), 0,0,sin(pose3d(2)/2));
  Eigen::Vector3f trans(pose3d(0),pose3d(1),0), transformedPoint, initPoint;
  for(int i=0;i<pcin.points.size();i++)
  {
    initPoint<<pcin.points[i].x, pcin.points[i].y,0;
    geometry_msgs::Point32 tempoint;
    transformedPoint=q.toRotationMatrix().transpose()*initPoint+trans;
    tempoint.x=transformedPoint(0); tempoint.y=-transformedPoint(1);tempoint.z=0;
    pcout.points.push_back(tempoint); 
  }
     pcout.header.frame_id=target_frame;    
}
*/

}







#endif