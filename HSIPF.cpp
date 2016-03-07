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


#include "HSIPF.h"

HSIPF::HSIPF()
{

}

HSIPF::~HSIPF()
{

}

float HSIPF::VectorAngle(Eigen::Vector3f veca, Eigen::Vector3f vecb)
{
 float noma = sqrt(veca[0]*veca[0]+veca[1]*veca[1]+veca[2]*veca[2]);
 float nomb = sqrt(vecb[0]*vecb[0]+vecb[1]*vecb[1]+vecb[2]*vecb[2]);
 float costheta = (veca[0]*vecb[0] + veca[1]*vecb[1] + veca[2]*vecb[2])/(noma*nomb);
 return acos(costheta);
}


double HSIPF::computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}

pcl::PointCloud< PointType > HSIPF::readPointCloud(string pointcloudFileName)
{
  pcl::PointCloud<PointType> pointcloud;
  if(pointcloudFileName.find(".ply") == pointcloudFileName.length() - 4)
  {
    if (pcl::io::loadPLYFile<PointType> (pointcloudFileName, pointcloud) == -1) //* load the file
    {
      cerr << "can not read file " << pointcloudFileName << endl;
      exit(1);
    }
  }
  if(pointcloudFileName.find(".pcd") == pointcloudFileName.length() - 4)
  {
    if (pcl::io::loadPCDFile<PointType> (pointcloudFileName, pointcloud) == -1) //* load the file
    {
      cerr << "can not read file " << pointcloudFileName << endl;
      exit(1);
    }
  }
  Eigen::Vector4f sensor_origin;  
  Eigen::Quaternion<float> sensor_orientation;  
  sensor_origin = Eigen::Vector4f::Zero();  
  sensor_orientation = Eigen::Quaternionf::Identity ();
  pointcloud.sensor_origin_ = sensor_origin;
  pointcloud.sensor_orientation_ = sensor_orientation;
  return pointcloud;
}

pcl::PointCloud< pcl::Normal > HSIPF::calculateNormal(pcl::PointCloud< PointType > pointcloud, 
						      float radious)
{
  
  pcl::PointXYZ centerpoint1;
  centerpoint1.x = 0;
  centerpoint1.y = 0;
  centerpoint1.z = 0;
  #pragma omp parallel for
  for(int i = 0; i < pointcloud.size(); ++i)
  {
    centerpoint1.x += pointcloud.at(i).x;
    centerpoint1.y += pointcloud.at(i).y;
    centerpoint1.z += pointcloud.at(i).z;
  }
  centerpoint1.x /= pointcloud.size();
  centerpoint1.y /= pointcloud.size();
  centerpoint1.z /= pointcloud.size();
		
  pcl::PointCloud<pcl::Normal> normal;
  cout << "calculating normal" << endl;
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne1;
  ne1.setInputCloud (pointcloud.makeShared());
  ne1.setNumberOfThreads(8);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
  ne1.setSearchMethod (tree);
  // Use all neighbors in a sphere of radius 3cm
  ne1.setRadiusSearch (radious);
  //ne1.setViewPoint (centerpoint1.x, centerpoint1.y, centerpoint1.z);	
  // Compute the features
  ne1.compute (normal);
  return normal;
}

bool HSIPF::HSIPFCalculate(pcl::PointCloud< HSIPFFeature >& HSIPFDescriptor)
{
  
}
