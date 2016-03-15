#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include "HSIPF.h"
#include "ShpereBlocks.h"
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
  for(int i = 0; i < pointcloud1.size(); ++i)
  {
    pointcloudnormal1.at(i).normal_x = pointcloud1.at(i).normal_x;
    pointcloudnormal1.at(i).normal_y = pointcloud1.at(i).normal_y;
    pointcloudnormal1.at(i).normal_z = pointcloud1.at(i).normal_z;
  }
  
  pcl::PointCloud<PointType> pointcloud2;
  pcl::PointCloud<pcl::Normal> pointcloudnormal2;
  pointcloud2 = pointcloud1;
  pointcloudnormal2.resize(pointcloud2.size()); 
  for(int i = 0; i < pointcloud2.size(); ++i)
  {
    pointcloud2.at(i).x *= 1.5;
    pointcloud2.at(i).y *= 1.5;
    pointcloud2.at(i).z *= 1.5;
    pointcloudnormal2.at(i).normal_x = pointcloud2.at(i).normal_x;
    pointcloudnormal2.at(i).normal_y = pointcloud2.at(i).normal_y;
    pointcloudnormal2.at(i).normal_z = pointcloud2.at(i).normal_z;
  }
// 
//   pcl::visualization::PCLVisualizer mainview("pointcloud");  
//   mainview.setPosition(0,0);	
//   mainview.setBackgroundColor(0.9, 0.9, 0.9);
//   pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(pointcloud1.makeShared(), 108, 166, 205);
//   mainview.addPointCloud (pointcloud1.makeShared(), single_color, "newpointcloud1");
//   mainview.addPointCloudNormals<PointType, pcl::Normal>(pointcloud1.makeShared(), pointcloudnormal1.makeShared(), 10, 0.05, "normal2");
// 		
//   while (!mainview.wasStopped ())
//   {
//     mainview.spinOnce ();
//   } 	
  
  pcl::PointCloud<PointType> keypoints1, keypoints2;
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointType> grid;
  float leaf;			
  leaf = 0.01;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (pointcloud1.makeShared());
  grid.filter (keypoints1);
  cout << keypoints1.size() << endl;
  
  pcl::console::print_highlight ("Downsampling...\n");
  leaf = 0.01;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (pointcloud2.makeShared());
  grid.filter (keypoints2);
  cout << keypoints2.size() << endl;
  
  pcl::PointCloud< HSIPFFeature > HSIPFDescriptor1;
  hsipf.HSIPFInputNormal(pointcloudnormal1);
  hsipf.HSIPFInputPointCloud(pointcloud1);
  hsipf.HSIPFInputKeypoint(keypoints1);
  hsipf.HSIPFSetupAngle(PI/2);
  hsipf.HSIPFSetupSphereModel("./spheresmall.ply");
  //hsipf.HSIPFSetupSphereModel("./sphere.ply");
  hsipf.HSIPFCalculate(HSIPFDescriptor1);
  
  pcl::PointCloud< HSIPFFeature > HSIPFDescriptor2;
  hsipf.HSIPFInputNormal(pointcloudnormal2);
  hsipf.HSIPFInputPointCloud(pointcloud2);
  hsipf.HSIPFInputKeypoint(keypoints2);
  hsipf.HSIPFSetupAngle(PI/2);
  hsipf.HSIPFSetupSphereModel("./spheresmall.ply");
  //hsipf.HSIPFSetupSphereModel("./sphere.ply");
  hsipf.HSIPFCalculate(HSIPFDescriptor2);
  
  
  
  /**
   *   \note check boundary points 
   */
  /*pcl::visualization::PCLVisualizer mainviewboundaryPoints("boundaryPoints");  
  mainviewboundaryPoints.setPosition(0,0);	
  mainviewboundaryPoints.setBackgroundColor(0.9, 0.9, 0.9);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> single_colorboundaryPoints(boundaryPoints.makeShared(), 108, 166, 205);
  mainviewboundaryPoints.addPointCloud (boundaryPoints.makeShared(), single_colorboundaryPoints, "newpointcloud1");  
  pcl::PointCloud< PointType > onekeypoint;
  onekeypoint.push_back(pointcloud1.at(0));
  pcl::visualization::PointCloudColorHandlerCustom<PointType> keypointcolor(onekeypoint.makeShared(), 0, 0, 0);
  pcl::PointCloud<pcl::Normal> pointcloudnormal2,pointcloudnormal3;
  pointcloudnormal2.resize(boundaryPoints.size()); 
  for(int i = 0; i < boundaryPoints.size(); ++i)
  {
    pointcloudnormal2.at(i).normal_x = boundaryPoints.at(i).normal_x;
    pointcloudnormal2.at(i).normal_y = boundaryPoints.at(i).normal_y;
    pointcloudnormal2.at(i).normal_z = boundaryPoints.at(i).normal_z;
  }
  pointcloudnormal3.resize(1);
  pointcloudnormal3.at(0).normal_x = pointcloud1.at(0).normal_x;
  pointcloudnormal3.at(0).normal_y = pointcloud1.at(0).normal_y;
  pointcloudnormal3.at(0).normal_z = pointcloud1.at(0).normal_z;
  mainviewboundaryPoints.addPointCloud (onekeypoint.makeShared(), keypointcolor, "keypointcolor"); 
  mainviewboundaryPoints.addPointCloudNormals<PointType, pcl::Normal>(boundaryPoints.makeShared(), pointcloudnormal2.makeShared(), 1, 0.05, "normal2");
  mainviewboundaryPoints.addPointCloudNormals<PointType, pcl::Normal>(onekeypoint.makeShared(), pointcloudnormal3.makeShared(), 1, 0.5, "normal3");
  while (!mainviewboundaryPoints.wasStopped ())
  {
    mainviewboundaryPoints.spinOnce ();
  } 	
  */
  
  
  return 1;
}