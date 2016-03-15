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
#include <pcl/visualization/histogram_visualizer.h>
#include <iostream>
using namespace std;

#include "common.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <omp.h>
#include <pcl/common/pca.h>
#include "ShpereBlocks.h"
#include <time.h>

struct HSIPFFeature
{
  pcl::Histogram< DIM > descriptor;
};

class HSIPF
{

  float BoundaryAngle;
  pcl::PointCloud<PointType> pointcloud;
  pcl::PointCloud<PointType> keypoints;
  pcl::PointCloud<pcl::Normal> normal;
  string sphereModel;
  
public:
  HSIPF();
  pcl::PointCloud<PointType> readPointCloud(string pointcloudFileName);
  pcl::PointCloud<pcl::Normal> calculateNormal(pcl::PointCloud<PointType> pointcloud, 
					       float radious);
  float VectorAngle(Eigen::Vector3f veca, 
		   Eigen::Vector3f vecb);
  
  Eigen::Vector3f CrossProduct(Eigen::Vector3f veca, 
		   Eigen::Vector3f vecb);
  
  inline void HSIPFInputPointCloud(pcl::PointCloud<PointType> pointcloud);
  inline void HSIPFInputKeypoint(pcl::PointCloud<PointType> keypoints);
  inline void HSIPFInputNormal(pcl::PointCloud<pcl::Normal> normal);
  inline void HSIPFSetupAngle(float angle);
  inline void HSIPFSetupSphereModel(string spheremodel);
  bool HSIPFCalculate(pcl::PointCloud< HSIPFFeature > & HSIPFDescriptor);
  double computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr& cloud);
  Eigen::Matrix<float, 3, 3> RotationAboutVector(Eigen::Vector3f rotationAxis, float theta);
  pcl::PointCloud<PointType> getBoundaryPoints(PointType keypoint);
  pcl::PointCloud<PointType> getBoundaryPointsnew(PointType keypoint);
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

void HSIPF::HSIPFInputKeypoint(pcl::PointCloud< PointType > keypoints)
{
  this->keypoints = keypoints;
}

void HSIPF::HSIPFSetupSphereModel(string spheremodel)
{
  this->sphereModel = spheremodel;
}





#endif // HSIPF_H
