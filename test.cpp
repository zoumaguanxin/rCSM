#include "CorrelativeMatch.hpp"
#include "csm_ros.hpp"

int main(int argc,char** argv)
{
 ros::init(argc,argv,"csm");
 ros::NodeHandle n;
 CSM_ROS::roscsm rcsm(n);
 rcsm.init();
 rcsm.run();
  
}