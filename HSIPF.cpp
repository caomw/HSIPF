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

Eigen::Matrix<float, 3, 3> HSIPF::RotationAboutVector(Eigen::Vector3f rotationAxis, float theta)
{
  Eigen::Matrix<float, 3, 3> rotationMatrix;
  float norm = rotationAxis.norm();
  Eigen::Vector3f normlizedRotationAxis = rotationAxis / norm;
  rotationMatrix(0, 0) = cos(theta)+normlizedRotationAxis(0)*normlizedRotationAxis(0)*(1-cos(theta));
  rotationMatrix(0, 1) = normlizedRotationAxis(0)*normlizedRotationAxis(1)*(1-cos(theta))-normlizedRotationAxis(2)*sin(theta);
  rotationMatrix(0, 2) = normlizedRotationAxis(1)*sin(theta)+normlizedRotationAxis(0)*normlizedRotationAxis(2)*(1-cos(theta));
  
  rotationMatrix(1, 0) = normlizedRotationAxis(2)*sin(theta)+normlizedRotationAxis(0)*normlizedRotationAxis(1)*(1-cos(theta));
  rotationMatrix(1, 1) = cos(theta)+normlizedRotationAxis(1)*normlizedRotationAxis(1)*(1-cos(theta));
  rotationMatrix(1, 2) = -normlizedRotationAxis(0)*sin(theta)+normlizedRotationAxis(1)*normlizedRotationAxis(2)*(1-cos(theta));
  
  rotationMatrix(2, 0) = -normlizedRotationAxis(1)*sin(theta)+normlizedRotationAxis(0)*normlizedRotationAxis(2)*(1-cos(theta));
  rotationMatrix(2, 1) = normlizedRotationAxis(0)*sin(theta)+normlizedRotationAxis(1)*normlizedRotationAxis(2)*(1-cos(theta));
  rotationMatrix(2, 2) = cos(theta)+normlizedRotationAxis(2)*normlizedRotationAxis(2)*(1-cos(theta));
  
  if(abs(rotationMatrix.determinant() - 1) < 0.0001)
    return rotationMatrix;
  else 
  {
    cerr << "the determinat of rotation matirx is not equal to 1, system exit(1)!!" << endl;
    exit(1);
  }
}


float HSIPF::VectorAngle(Eigen::Vector3f veca, Eigen::Vector3f vecb)
{
  float noma = sqrt(veca[0]*veca[0]+veca[1]*veca[1]+veca[2]*veca[2]);
  float nomb = sqrt(vecb[0]*vecb[0]+vecb[1]*vecb[1]+vecb[2]*vecb[2]);
  float costheta = (veca[0]*vecb[0] + veca[1]*vecb[1] + veca[2]*vecb[2])/(noma*nomb);
  if(costheta > 1.0000000)
  costheta = 0.999999999;
  if(costheta < -1.0000000)
  costheta = -0.99999999;	
  return acos(costheta);
}

Eigen::Vector3f HSIPF::CrossProduct(Eigen::Vector3f veca, Eigen::Vector3f vecb)
{
  Eigen::Vector3f crossVec;
  crossVec[0] = veca[1]*vecb[2] - vecb[1]*veca[2];
  crossVec[1] = veca[2]*vecb[0] - vecb[2]*veca[0];
  crossVec[2] = veca[0]*vecb[1] - vecb[0]*veca[1];
  return crossVec;
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

pcl::PointCloud< PointType > HSIPF::getBoundaryPoints(PointType keypoint)
{
  pcl::PointCloud< PointType > boundaryPoints, newPointcloud;
  pcl::PointCloud< PointType >::iterator iter, end;
  newPointcloud = this->pointcloud;
  Eigen::Vector3f keypointEig = keypoint.getArray3fMap();
  Eigen::Vector3f keypointNormal = keypoint.getNormalVector3fMap();
  for (iter = newPointcloud.begin(), end = newPointcloud.end(); iter != end; ++iter)
  {      
    PointType boundarypoint;
    float mindis = 10e10;
    bool pushflag = false;
    Eigen::Vector3f boundaryPointsCand = iter->getArray3fMap();    
    Eigen::Vector3f vectortmp = boundaryPointsCand - keypointEig;
    Eigen::Vector3f pointNormal1 = iter->getNormalVector3fMap();
    if(abs(VectorAngle(keypointNormal, pointNormal1) - this->BoundaryAngle) < 0.01)
    {
      pushflag = true;
      boundarypoint.x = iter->getArray3fMap()(0);
      boundarypoint.y = iter->getArray3fMap()(1);
      boundarypoint.z = iter->getArray3fMap()(2);
      boundarypoint.normal_x = iter->getNormalVector3fMap()(0);
      boundarypoint.normal_y = iter->getNormalVector3fMap()(1);
      boundarypoint.normal_z = iter->getNormalVector3fMap()(2);
      
      mindis = vectortmp.norm();
      pcl::PointCloud< PointType >::iterator iter2, end2;
      for (iter2 = newPointcloud.begin(), end2 = newPointcloud.end(); iter2 != end2; ++iter2)
      { 	
	Eigen::Vector3f boundaryPointsCand2 = iter2->getArray3fMap();
	Eigen::Vector3f vectortmp2 = boundaryPointsCand2 - keypointEig;
	Eigen::Vector3f pointNormal = iter2->getNormalVector3fMap();
	if(VectorAngle(vectortmp, vectortmp2) < 0.0001)
	{
	  if(abs(VectorAngle(keypointNormal, pointNormal) - this->BoundaryAngle) < 0.01)
	  {	    
	    if(vectortmp2.norm() < mindis)
	    {
	      boundarypoint.x = boundaryPointsCand2(0);
	      boundarypoint.y = boundaryPointsCand2(1);
	      boundarypoint.z = boundaryPointsCand2(2);
	      boundarypoint.normal_x = pointNormal(0);
	      boundarypoint.normal_y = pointNormal(1);
	      boundarypoint.normal_z = pointNormal(2);
	      mindis = vectortmp2.norm();
	    }
	  }
	  newPointcloud.erase(iter2);
	}    
      }          
    }  
    newPointcloud.erase(iter);
    if(pushflag)
      boundaryPoints.push_back(boundarypoint);
    if(newPointcloud.size() == 0)
      break;
    //cerr << newPointcloud.size() << " " << boundaryPoints.size() << endl;
  }
  return boundaryPoints;
}


bool HSIPF::HSIPFCalculate(pcl::PointCloud< HSIPFFeature >& HSIPFDescriptor)
{
  pcl::visualization::PCLVisualizer mainview("pointcloud");  
  mainview.setPosition(0,0);	
  mainview.setBackgroundColor(0.9, 0.9, 0.9);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(this->pointcloud.makeShared(), 108, 166, 205);
  mainview.addPointCloud (this->pointcloud.makeShared(), single_color, "newpointcloud1");
  Eigen::Vector3f pt1, pt2;
  
  vector< pair < vector <Eigen::Vector3f>, Eigen::Vector3f > >  trianglePoints, newtrianglePoints;
  ShpereBlocks shpereblock;
  shpereblock.TriangleBlocks(trianglePoints);  
  newtrianglePoints = trianglePoints;
  float alpha = PI/30;
  int K = PI/3/alpha;  
  for(int i = 0; i < this->keypoints.size(); ++i)
  {
    vector< pcl::PointCloud<PointType> > pointsinTrianle(trianglePoints.size() * K);   //boundary points in each triangle
    pcl::PointCloud< PointType > boundaryPoints = getBoundaryPoints(keypoints.at(i));    
    Eigen::Vector3f SO;
    SO << 0, 1, 0;
    Eigen::Vector3f PN = this->normal.at(i).getNormalVector3fMap();
    Eigen::Vector3f SOcrossPN = CrossProduct(SO, PN);
    //cerr << SOcrossPN << endl;
    float thetaSOPN = VectorAngle(SO, PN);
    //cerr << thetaSOPN << endl;
    Eigen::Matrix<float, 3, 3> roMatAboutSOcrossPN = RotationAboutVector(SOcrossPN, thetaSOPN);   

    Eigen::Vector3f translation = keypoints.at(i).getArray3fMap();
    //omp_set_num_threads(2);
    //#pragma omp parallel for
    for(int j = 0; j < K; ++j)
    {
      stringstream sk;
      sk << j;
      Eigen::Matrix<float, 3, 3> roMatAboutPN = RotationAboutVector(PN, j*alpha);
      for(int indt = 0; indt < trianglePoints.size(); ++indt)
      {
	newtrianglePoints.at(indt).first.at(0) = (roMatAboutPN*roMatAboutSOcrossPN*trianglePoints.at(indt).first.at(0)+translation);
	newtrianglePoints.at(indt).first.at(1) = (roMatAboutPN*roMatAboutSOcrossPN*trianglePoints.at(indt).first.at(1)+translation);
	newtrianglePoints.at(indt).first.at(2) = (roMatAboutPN*roMatAboutSOcrossPN*trianglePoints.at(indt).first.at(2)+translation);
	newtrianglePoints.at(indt).second = roMatAboutPN*roMatAboutSOcrossPN*trianglePoints.at(indt).second+translation;
	
	pt1 = translation;
	pt2 << 0, 1, 0;
	pt2 = roMatAboutPN*roMatAboutSOcrossPN*pt2+translation;
	
	stringstream sindt;
	sindt << indt;
	pcl::PointXYZ ppt1, ppt2;
	ppt1.x = pt1(0);
	ppt1.y = pt1(1);
	ppt1.z = pt1(2);
	ppt2.x = pt2(0);
	ppt2.y = pt2(1);
	ppt2.z = pt2(2);
	mainview.addLine(ppt1, ppt2, sk.str()+sindt.str()+"line");      
      }
       
      for(int indn = 0; indn < newtrianglePoints.size(); ++ indn)
      {
	pcl::PointCloud<PointType> pointcloudtmp,centers;
	pointcloudtmp.resize(3);
	pointcloudtmp.at(0).x = newtrianglePoints.at(indn).first.at(0)[0];
	pointcloudtmp.at(0).y = newtrianglePoints.at(indn).first.at(0)[1];
	pointcloudtmp.at(0).z = newtrianglePoints.at(indn).first.at(0)[2];
	pointcloudtmp.at(1).x = newtrianglePoints.at(indn).first.at(1)[0];
	pointcloudtmp.at(1).y = newtrianglePoints.at(indn).first.at(1)[1];
	pointcloudtmp.at(1).z = newtrianglePoints.at(indn).first.at(1)[2];
	pointcloudtmp.at(2).x = newtrianglePoints.at(indn).first.at(2)[0];
	pointcloudtmp.at(2).y = newtrianglePoints.at(indn).first.at(2)[1];
	pointcloudtmp.at(2).z = newtrianglePoints.at(indn).first.at(2)[2];
	
	centers.resize(1);
	centers.at(0).x = newtrianglePoints.at(indn).second[0];
	centers.at(0).y = newtrianglePoints.at(indn).second[1];
	centers.at(0).z = newtrianglePoints.at(indn).second[2];
	
	stringstream sindn;
	sindn << indn;
	pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(pointcloudtmp.makeShared(), 0, 0, 0);
	mainview.addPointCloud (pointcloudtmp.makeShared(), single_color, sk.str()+sindn.str()+"points");
	mainview.addText3D(sindn.str(),centers.at(0),0.05,1,1,1,sk.str()+sindn.str()+"text");	
      }  
      
      for(int indn = 0; indn < newtrianglePoints.size(); ++ indn)
      {
	Eigen::Vector3f point1, point2, point3;
	point1 = newtrianglePoints.at(indn).first.at(0);
	point2 = newtrianglePoints.at(indn).first.at(1);
	point3 = newtrianglePoints.at(indn).first.at(2);
	Eigen::Vector3f O;
	Eigen::Vector3f M = keypoints.at(i).getVector3fMap();
	for(int indb = 0; indb < boundaryPoints.size(); ++indb)
	{
	  
	  Eigen::Vector3f Vp = CrossProduct(point2-point1, point3-point1);
	  Eigen::Vector3f N = point1;
	  Eigen::Vector3f Vl = boundaryPoints.at(indb).getVector3fMap() - M;
	  
	  float t = ((Vp.transpose() * (N-M)) / (Vp.transpose() * Vl)) [0];
	  O(0) = M(0)+Vl(0)*t;
	  O(1) = M(1)+Vl(1)*t;
	  O(2) = M(2)+Vl(2)*t;
	  
	  if(abs(VectorAngle(point1-O, point2-O) + VectorAngle(point2-O, point3-O) + VectorAngle(point1-O, point3-O) - 2*PI) < 0.0001)
	  {
	    pointsinTrianle.at(j*trianglePoints.size() + indn).push_back(boundaryPoints.at(indb));
	  }
	}	
      }      
    }
//     while (!mainview.wasStopped ())
//     {
//       mainview.spinOnce ();
//     }

    
    pcl::PointCloud<PointType> allpoints;
    HSIPFFeature his;
    float hisNorm = 0;
    for(int indhis = 0; indhis < DIM; ++indhis)
    {
      his.descriptor.histogram[indhis] = 0;
    }
    for(int indpt = 0; indpt < pointsinTrianle.size(); ++indpt)
    {
//       pcl::visualization::PCLVisualizer view2("pointcloud1");  
//       view2.setPosition(0,0);	
//       view2.setBackgroundColor(0.9, 0.9, 0.9);
//       pcl::PointCloud<PointType> onekeypoint;
//       onekeypoint.push_back(keypoints.at(i));
//       pcl::visualization::PointCloudColorHandlerCustom<PointType> single_colorview3(onekeypoint.makeShared(), 0,0,0);
//       view2.addPointCloud (onekeypoint.makeShared(), single_colorview3, "keypoint");
      if(pointsinTrianle.at(indpt).size() > 5)  //5 could be a problem,
      {
	//cout << pointsinTrianle.at(indpt).at(0).getVector3fMap()<< endl;
	//cout << pointsinTrianle.at(indpt).at(1).getVector3fMap()<< endl;
	//cout << pointsinTrianle.at(indpt).at(2).getVector3fMap()<< endl;
	pcl::PCA<PointType> pca;
	pca.setInputCloud(pointsinTrianle.at(indpt).makeShared());
	Eigen::Vector3f eigenvalues = pca.getEigenValues();
	Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
	Eigen::Vector3f keypointNormal = keypoints.at(i).getNormalVector3fMap();
	//cout << eigenvectors << endl;
	//cout << eigenvalues << endl;
	Eigen::Vector3f V1 = eigenvectors.col(0);
	Eigen::Vector3f V2 = eigenvectors.col(1);
	Eigen::Vector3f V3 = eigenvectors.col(2);
	if((eigenvalues(0) + eigenvalues(1))/(eigenvalues(0) + eigenvalues(1) + eigenvalues(2))>0.95) //the first and second princeple have almost all of the information
	{
	  float AngV1On = VectorAngle(V1,keypointNormal);
	  float AngV2On = VectorAngle(V2,keypointNormal);
	  his.descriptor.histogram[int(AngV1On/alpha)] ++;
	  his.descriptor.histogram[int(AngV2On/alpha)] ++;
	}
	
	stringstream sindn;
	sindn << indpt;
	//cerr << sindn.str() << endl;
	
// 	for(int tmpi = 0; tmpi < pointsinTrianle.at(indpt).size(); ++tmpi)
// 	  allpoints.push_back(pointsinTrianle.at(indpt).at(tmpi));
// 	pcl::visualization::PointCloudColorHandlerCustom<PointType> single_colorview2(allpoints.makeShared(), 108*indpt%255, 166*indpt%255, 205*indpt%255);
// 	view2.addPointCloud (allpoints.makeShared(), single_colorview2, sindn.str()+"newpointcloud1");
      } 
      /*while (!view2.wasStopped ())
      {
	view2.spinOnce ();
      }  */    
    }
    for(int indhis = 0; indhis < DIM; ++indhis)
    {
      hisNorm += his.descriptor.histogram[indhis] * his.descriptor.histogram[indhis];      
    }
    hisNorm = sqrt(hisNorm);
    for(int indhis = 0; indhis < DIM; ++indhis)
    {
      his.descriptor.histogram[indhis]/=hisNorm;
      cout << his.descriptor.histogram[indhis] << " ";
    }
    cout << endl;
    HSIPFDescriptor.push_back(his);
    exit(1);
  }
}
