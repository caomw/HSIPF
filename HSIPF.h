/*
    Copyright 2016 <copyright holder> <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef HSIPF_H
#define HSIPF_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <iostream>
using namespace std;

#include "common.h"

#include <omp.h>

struct HSIPFFeature
{
  pcl::Histogram< DIM > descriptor;
};

class HSIPF
{

  float BoundaryAngle;
  pcl::PointCloud<PointType> pointcloud;
  pcl::PointCloud<pcl::Normal> normal;
  
public:
  HSIPF();
  pcl::PointCloud<PointType> readPointCloud(string pointcloudFileName);
  pcl::PointCloud<pcl::Normal> calculateNormal(pcl::PointCloud<PointType> pointcloud, 
					       float radious);
  float VectorAngle(Eigen::Vector3f veca, 
		   Eigen::Vector3f vecb);
  
  Eigen::Vector3f CorssProduct(Eigen::Vector3f veca, 
		   Eigen::Vector3f vecb);
  
  inline void HSIPFInputPointCloud(pcl::PointCloud<PointType> pointcloud);
  inline void HSIPFInputNormal(pcl::PointCloud<pcl::Normal> normal);
  inline void HSIPFSetupAngle(float angle);
  bool HSIPFCalculate(pcl::PointCloud< HSIPFFeature > & HSIPFDescriptor);
  double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr& cloud);
  virtual ~HSIPF();
  
};

void HSIPF::HSIPFSetupAngle(float angle)
{
  this->BoundaryAngle = angle;
}
void HSIPF::HSIPFInputPointCloud(pcl::PointCloud< PointType > pointcloud)
{
  this->pointcloud = pointcloud;
}
void HSIPF::HSIPFInputNormal(pcl::PointCloud< pcl::Normal > normal)
{
  this->normal = normal;
}



#endif // HSIPF_H
