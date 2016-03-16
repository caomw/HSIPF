#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
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
    pointcloud2.at(i).x += 0.5;
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
  leaf = 0.02;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (pointcloud2.makeShared());
  grid.filter (keypoints2);
  cout << keypoints2.size() << endl;
  
  pcl::visualization::PCLVisualizer keypoints("keypoints");  
  keypoints.setPosition(0,0);	
  keypoints.setBackgroundColor(0.9, 0.9, 0.9);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> keypoints1color(keypoints1.makeShared(), 108, 166, 205);
  keypoints.addPointCloud (keypoints1.makeShared(), keypoints1color, "keypoints1");  
  pcl::PointCloud< PointType > onekeypoint;
  onekeypoint.push_back(pointcloud1.at(0));
  pcl::visualization::PointCloudColorHandlerCustom<PointType> keypoints2color(keypoints2.makeShared(), 255, 0, 0);
  keypoints.addPointCloud (keypoints2.makeShared(), keypoints2color, "keypoints2"); 
  while (!keypoints.wasStopped ())
  {
    keypoints.spinOnce ();
  } 	
    
  
  pcl::PointCloud< HSIPFFeature30 > HSIPFDescriptor1;
  
  
//   hsipf.HSIPFInputNormal(pointcloudnormal1);
//   hsipf.HSIPFInputPointCloud(pointcloud1);
//   hsipf.HSIPFInputKeypoint(keypoints1);
//   hsipf.HSIPFSetupAngle(PI/2);
//   hsipf.HSIPFSetupSphereModel("./spheresmall.ply");
//   //hsipf.HSIPFSetupSphereModel("./sphere.ply");
//   hsipf.HSIPFCalculate(HSIPFDescriptor1);
//   hsipf.saveHSIPFDescriptor(HSIPFDescriptor1, "feature1.txt");
  
  pcl::PointCloud< HSIPFFeature30 > HSIPFDescriptor2;
  
  
//   hsipf.HSIPFInputNormal(pointcloudnormal2);
//   hsipf.HSIPFInputPointCloud(pointcloud2);
//   hsipf.HSIPFInputKeypoint(keypoints2);
//   hsipf.HSIPFSetupAngle(PI/2);
//   hsipf.HSIPFSetupSphereModel("./spheresmall.ply");
//   //hsipf.HSIPFSetupSphereModel("./sphere.ply");
//   hsipf.HSIPFCalculate(HSIPFDescriptor2);  
//   hsipf.saveHSIPFDescriptor(HSIPFDescriptor2, "feature2.txt");
//   exit(1);
  
  hsipf.loadHSIPFDescriptor(HSIPFDescriptor1, "feature1.txt");
  hsipf.loadHSIPFDescriptor(HSIPFDescriptor2, "feature2.txt");

  hsipf.HSIPFSetupCorresSourceFeature(HSIPFDescriptor1);
  hsipf.HSIPFSetupCorresTargetFeature(HSIPFDescriptor2);
  pcl::Correspondences all_correspondences;
  //hsipf.determineCorrespondences(all_correspondences);   
  hsipf.determineCorrespondences2(all_correspondences);
  cout << "Found "<<all_correspondences.size()<<" total feature correspondences.\n";
  
  
  pcl::visualization::PCLVisualizer viewer3("HSIPF corresponding");  
  viewer3.setBackgroundColor(0.9, 0.9, 0.9);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color5(pointcloud1.makeShared(), 108, 166, 205);
  viewer3.addPointCloud (pointcloud1.makeShared(), single_color5, "newpointcloud1");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color6(pointcloud2.makeShared(), 108, 166, 205);
  viewer3.addPointCloud (pointcloud2.makeShared(), single_color6, "newpointcloud2");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> single_colorkeypoint1(keypoints1.makeShared(), 255, 0, 0);
  viewer3.addPointCloud (keypoints1.makeShared(), single_colorkeypoint1, "keypoints1");	
  pcl::visualization::PointCloudColorHandlerCustom<PointType> single_colorkeypoint2(keypoints2.makeShared(), 255, 0, 0);
  viewer3.addPointCloud (keypoints2.makeShared(), single_colorkeypoint2, "keypoints2");	
  viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints1");
  viewer3.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints2");
  viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "newpointcloud1");
  viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "newpointcloud2");	
  viewer3.setSize (1280, 600);
  viewer3.setPosition(0,0);			
  viewer3.resetCameraViewpoint();
  viewer3.resetCamera();
  for (size_t i = 0; i < all_correspondences.size (); ++i)
  {
    pcl::PointXYZ tmppoint1;
    tmppoint1.x = keypoints1.at(all_correspondences[i].index_query).x;
    tmppoint1.y = keypoints1.at(all_correspondences[i].index_query).y;
    tmppoint1.z = keypoints1.at(all_correspondences[i].index_query).z; 

    pcl::PointXYZ tmppoint2;
    tmppoint2.x = keypoints2.at(all_correspondences[i].index_match).x;
    tmppoint2.y = keypoints2.at(all_correspondences[i].index_match).y;
    tmppoint2.z = keypoints2.at(all_correspondences[i].index_match).z;
    stringstream ss;
    ss << i;
    string sss;
    ss >> sss;
  	
    viewer3.addLine<pcl::PointXYZ>(tmppoint1, tmppoint2, sss);
  }
  cout << "good correspondences: " << all_correspondences.size() << endl;
  while (!viewer3.wasStopped ())
  {
    viewer3.spinOnce ();
  }
  
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