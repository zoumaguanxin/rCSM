/*
 * Copyright 2019 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#include <iostream>
#include <vector>
#include <Eigen/Dense>




namespace sensor{
	

	struct Position{
		Position(double x_,double y_,double z_):x(x_),y(y_),z(z_){}
		Position():x(0.),y(0.),z(0.){}
		double x,y,z;		
		template<size_t N> 
		Eigen::Matrix<float,N,1> head() const		
		{
			 Eigen::Matrix<float,N,1> result;
			 assert(N<=3);
			 if(N==2)
			 {
				 result<<x,y;
				 return result;
			}
			else if(N==1)
			{
				result<<x;
				return result;
			}
			else
			{
				result<<x,y,z;
			}
		}
	};
	


struct RangePoint
{
	Position position;
};


typedef	std::vector<RangePoint> PointCloud;




sensor::PointCloud TransformPointCloud(
        PointCloud mvPcd, Eigen::Matrix3f R);

}//namespace sensor
#endif // POINTCLOUD_H
