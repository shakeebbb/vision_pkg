
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int point2collision(Mat& projectionMatrix, Mat& distortionVector, vector<Point3f> worldPoints, Mat& imgDepth)
{

    // Decompose the projection matrix into:
  Mat K(3,3,cv::DataType<double>::type); // intrinsic parameter matrix
  Mat rvec(3,3,cv::DataType<double>::type); // rotation matrix
  
  Mat Thomogeneous(4,1,cv::DataType<double>::type); // translation vector

  decomposeProjectionMatrix(projectionMatrix, K, rvec, Thomogeneous);

cout << "........................." << endl;

for(int i = 0; i < 3; i++) // Projection/camera matrix: For debugging
{
	for(int j = 0; j < 4; j++)
	cout << projectionMatrix.at<double>(i,j) << " ";
	cout << endl;
}

for(int i = 0; i < 3; i++) // Camera intrinsic matrix: For debugging
{
	for(int j = 0; j < 3; j++)
	cout << K.at<double>(i,j) << " ";
	cout << endl;
}

for(int i = 0; i < 3; i++) // Camera intrinsic matrix: For debugging
{
	for(int j = 0; j < 3; j++)
	cout << rvec.at<double>(i,j) << " ";
	cout << endl;
}

	for(int j = 0; j < 4; j++)
	cout << Thomogeneous.at<double>(j,0) << " ";
	cout << endl;

  Mat T(3,1,cv::DataType<double>::type); // translation vector
  //cv::Mat T;

	for(int i = 0 ; i < 3 ; i++)
	T.at<double>(i,0) = Thomogeneous.at<double>(i,0) / Thomogeneous.at<double>(3,0);

  // cv::convertPointsFromHomogeneous(Thomogeneous, T);

  cout << "K: " << K << endl;
  cout << "rvec: " << rvec << endl;
  cout << "T: " << T << endl;
  
  // Create zero distortion

  vector<Point2f> projectedPoints;

  Mat rvecR(3,1,cv::DataType<double>::type);//rodrigues rotation matrix
  Rodrigues(rvec,rvecR);
  
  projectPoints(worldPoints, rvecR, T, K, distortionVector, projectedPoints);

	for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
    std::cout << "World point: " << worldPoints[i] << " Projected to " << projectedPoints[i] << "Depth : " << imgDepth.at<float>(projectedPoints[i].x, projectedPoints[i].y) << endl;
		
		if(worldPoints[i].z < imgDepth.at<float>(projectedPoints[i]))
		{
		cout << "World Point " << i+1 << " Safe" << endl;
		circle(imgDepth, projectedPoints[i], 10, Scalar(255,0,0), 5);
		}
		else
		{
		cout << "World Point " << i+1 << " Under Collision / Occluded" << endl;
		circle(imgDepth, projectedPoints[i], 10, Scalar(0,0,0), 5);
		}
		
    }

return 0;
}

