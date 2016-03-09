#include "ShpereBlocks.h"

/**
 *   this class must work with file sphere.ply * 
 */

ShpereBlocks::ShpereBlocks()
{

}

ShpereBlocks::~ShpereBlocks()
{

}

float ShpereBlocks::vectorNorm(Eigen::Vector3f vec)
{
  return sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
}


void ShpereBlocks::TriangleBlocks()
{
  pcl::PointCloud<PointType> pointcloud1;
  string pointcloudFileName = "./sphere.ply";
  pointcloud1 = readPointCloud(pointcloudFileName);
  pcl::PointCloud<pcl::Normal> pointcloudnormal1;
  pointcloudnormal1.resize(pointcloud1.size());
  for(int i = 0; i < pointcloud1.size(); ++i)
  {
    pointcloudnormal1.at(i).normal_x = pointcloud1.at(i).normal_x;
    pointcloudnormal1.at(i).normal_y = pointcloud1.at(i).normal_y;
    pointcloudnormal1.at(i).normal_z = pointcloud1.at(i).normal_z;
  }
  vector< pair < vector <Eigen::Vector3f>, Eigen::Vector3f > > triangles;  //n triangles
					   //3 points in one TriangleBlocks
  triangles.resize(0);
					   
  float minimumLength = 10e10f;
  Eigen::Vector3f vector3feigen1,vector3fcenter;
  vector3feigen1[0] = pointcloud1.at(0).x;
  vector3feigen1[1] = pointcloud1.at(0).y;
  vector3feigen1[2] = pointcloud1.at(0).z;  
  vector3fcenter[0] = 0;
  vector3fcenter[1] = 0;
  vector3fcenter[2] = 0; 
  //cout << pointcloud1.size() << endl;
  for(int i = 1; i < pointcloud1.size(); ++i)
  {
    Eigen::Vector3f vector3feigen2;
    vector3feigen2[0] = pointcloud1.at(i).x;
    vector3feigen2[1] = pointcloud1.at(i).y; 
    vector3feigen2[2] = pointcloud1.at(i).z;
    float tmplength = (vector3feigen1-vector3feigen2).norm();
    if(tmplength < minimumLength)
    {
      minimumLength = tmplength;
    }
  }  
  cerr << minimumLength << endl;
  
  //omp_set_num_threads(8); 
  //#pragma omp parallel for
  for(int i = 0; i < pointcloud1.size(); ++i)
  {    
    Eigen::Vector3f vector3ftmp1;
    vector3ftmp1[0] = pointcloud1.at(i).x;
    vector3ftmp1[1] = pointcloud1.at(i).y; 
    vector3ftmp1[2] = pointcloud1.at(i).z;
    for(int j = 0; j < pointcloud1.size(); ++j)
    {
      Eigen::Vector3f vector3ftmp2;
      vector3ftmp2[0] = pointcloud1.at(j).x;
      vector3ftmp2[1] = pointcloud1.at(j).y; 
      vector3ftmp2[2] = pointcloud1.at(j).z;
      for(int p = 0; p < pointcloud1.size(); ++p)
      {
	Eigen::Vector3f vector3ftmp3;
	vector3ftmp3[0] = pointcloud1.at(p).x;
	vector3ftmp3[1] = pointcloud1.at(p).y; 
	vector3ftmp3[2] = pointcloud1.at(p).z;
	if(i != j && i != p && j != p)
	{
	  float edge1 = (vector3ftmp1 - vector3ftmp2).norm();
	  float edge2 = (vector3ftmp1 - vector3ftmp3).norm();
	  float edge3 = (vector3ftmp2 - vector3ftmp3).norm();
	  static int count  =0;
// 	  if(edge1  < 0.62  && edge2 < 0.62 && edge3 < 0.62)
// 	  {
// 	     cout <<count << ":" << edge1 << "\t" << edge2 << "\t" << edge3 << endl;
// 	      count ++;
// 	  }
	  if(edge1  < 0.62  && edge2 < 0.62 && edge3 < 0.62)
	  {
	    Eigen::Vector3f fromCenterToTriangleDis;
	    fromCenterToTriangleDis = (vector3ftmp1 + vector3ftmp2 + vector3ftmp3)/3;
	    if(triangles.size() == 0)
	    {
	      vector <Eigen::Vector3f> threepoints;
	      threepoints.push_back(vector3ftmp1);
	      threepoints.push_back(vector3ftmp2);
	      threepoints.push_back(vector3ftmp3);	      
	      pair<vector <Eigen::Vector3f>, Eigen::Vector3f> onetriangle;
	      onetriangle.first = threepoints;
	      onetriangle.second = fromCenterToTriangleDis;
	      triangles.push_back(onetriangle);
	    }
	    int pointcount = 0;
	    for(int q = 0; q < triangles.size(); ++q)
	    {
	      double a = triangles.at(q).second.dot(fromCenterToTriangleDis);
	      double b = vectorNorm(triangles.at(q).second)* vectorNorm(fromCenterToTriangleDis);
	      double ratio = a/b;
	      if(ratio > 1.0000000)
		ratio = 0.999999999;
	      if(ratio < -1.0000000)
		ratio = -0.99999999;	      
	      if(abs(acos(ratio)) < 0.001 && (triangles.at(q).second-fromCenterToTriangleDis).norm() < 0.001)
	      {
		pointcount ++;
	      }
	    }
	    if(pointcount == 0)
	    {
	      vector <Eigen::Vector3f> threepoints;
	      threepoints.push_back(vector3ftmp1);
	      threepoints.push_back(vector3ftmp2);
	      threepoints.push_back(vector3ftmp3);	      
	      pair<vector <Eigen::Vector3f>, Eigen::Vector3f> onetriangle;
	      onetriangle.first = threepoints;
	      onetriangle.second = fromCenterToTriangleDis;
	      triangles.push_back(onetriangle);
	    }
	  }	  
	}
      }
    }
  }
  
  cout << triangles.size() << endl;
  pcl::visualization::PCLVisualizer mainview("pointcloud");  
  mainview.setPosition(0,0);	
  mainview.setBackgroundColor(0.9, 0.9, 0.9);
  for(int i = 0; i < triangles.size(); ++ i)
  {
    pcl::PointCloud<PointType> pointcloud,centers;
    pointcloud.resize(3);
    pointcloud.at(0).x = triangles.at(i).first.at(0)[0];
    pointcloud.at(0).y = triangles.at(i).first.at(0)[1];
    pointcloud.at(0).z = triangles.at(i).first.at(0)[2];
    pointcloud.at(1).x = triangles.at(i).first.at(1)[0];
    pointcloud.at(1).y = triangles.at(i).first.at(1)[1];
    pointcloud.at(1).z = triangles.at(i).first.at(1)[2];
    pointcloud.at(2).x = triangles.at(i).first.at(2)[0];
    pointcloud.at(2).y = triangles.at(i).first.at(2)[1];
    pointcloud.at(2).z = triangles.at(i).first.at(2)[2];
    
    centers.resize(1);
    centers.at(0).x = triangles.at(i).second[0];
    centers.at(0).y = triangles.at(i).second[1];
    centers.at(0).z = triangles.at(i).second[2];
    
    stringstream s;
    s << i;
    pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(pointcloud.makeShared(), 0, 0, 0);
    mainview.addPointCloud (pointcloud.makeShared(), single_color, s.str()+"points");
  
    //pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color2(centers.makeShared(), 255, 0, 0);
    //mainview.addPointCloud (centers.makeShared(), single_color2, s.str()+"centers");
    mainview.addText3D(s.str(),centers.at(0),0.05,1,1,1,s.str()+"text");
  }
		
  while (!mainview.wasStopped ())
  {
    mainview.spinOnce ();
  } 	
}


pcl::PointCloud< PointType > ShpereBlocks::readPointCloud(string pointcloudFileName)
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