#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include "HSIPF.h"
#include "common.h"
using namespace std;

int main(int argc, char ** argv)
{
  HSIPF hsipf;
  string pointcloudFileName = argv[1];
  cout << "Loading point cloud File name: " << pointcloudFileName << endl;
  pcl::PointCloud<PointType> pointcloud1;
  pointcloud1 = hsipf.readPointCloud(pointcloudFileName);
  pcl::PointCloud<pcl::Normal> pointcloudnormal1;
  //pointcloudnormal1 = hsipf.calculateNormal(pointcloud1, 0.02f);
  pointcloudnormal1.resize(pointcloud1.size());
  
  
  
  omp_set_num_threads(2); 
  #pragma omp parallel for
  for(int i = 0; i < pointcloud1.size(); ++i)
  {
	  pointcloudnormal1.at(i).normal_x = pointcloud1.at(i).normal_x;
	  pointcloudnormal1.at(i).normal_y = pointcloud1.at(i).normal_y;
	  pointcloudnormal1.at(i).normal_z = pointcloud1.at(i).normal_z;
  }

  pcl::visualization::PCLVisualizer mainview("pointcloud");  
  mainview.setPosition(0,0);	
  mainview.setBackgroundColor(0.9, 0.9, 0.9);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(pointcloud1.makeShared(), 108, 166, 205);
  mainview.addPointCloud (pointcloud1.makeShared(), single_color, "newpointcloud1");
  mainview.addPointCloudNormals<PointType, pcl::Normal>(pointcloud1.makeShared(), pointcloudnormal1.makeShared(), 10, 0.05, "normal2");
		
  while (!mainview.wasStopped ())
  {
    mainview.spinOnce ();
  } 	
  
  return 1;
}