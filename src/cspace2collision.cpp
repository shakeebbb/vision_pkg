// Takes a query point in camera coordinate frame, makes a rectangular sheild infront of it, projects this shield and checks each point in the shield for collision is depth space


#include "local_functions.h"

using namespace cv;
using namespace std;

int cspace2collision(Point3f queryPosition, float safetyRadius, Mat& projectionMatrix, Mat& distortionVector, Mat& imgDepth, Mat& imgRgb)
{

// Alters imgRgb : Dont for reuse, only for visualization

int isInCollision = 0;
int numCollisionPoints = 0;
vector<Point3f> rectanglePoints_v;

rectanglePoints_v.push_back( queryPosition + Point3f(0, 0, safetyRadius)); // Center point
rectanglePoints_v.push_back( queryPosition + Point3f(-safetyRadius, -safetyRadius, safetyRadius)); // Top-Left point
rectanglePoints_v.push_back( queryPosition + Point3f(safetyRadius, -safetyRadius, safetyRadius)); // Top-Right point
rectanglePoints_v.push_back( queryPosition + Point3f(safetyRadius, safetyRadius, safetyRadius)); // Bottom-Right point
rectanglePoints_v.push_back( queryPosition + Point3f(-safetyRadius, safetyRadius, safetyRadius)); // Bottom-Left point

    // Decompose the projection matrix into:
  Mat K(3,3,cv::DataType<double>::type); // intrinsic parameter matrix
  Mat rvec(3,3,cv::DataType<double>::type); // rotation matrix
  
  Mat Thomogeneous(4,1,cv::DataType<double>::type); // translation vector

  decomposeProjectionMatrix(projectionMatrix, K, rvec, Thomogeneous);
  
  Mat T(3,1,cv::DataType<double>::type); // translation vector
  //cv::Mat T;

	for(int i = 0 ; i < 3 ; i++)
	T.at<double>(i,0) = Thomogeneous.at<double>(i,0) / Thomogeneous.at<double>(3,0);
	
	vector<Point2f> projectedPoints;

  Mat rvecR(3,1,cv::DataType<double>::type);//rodrigues rotation matrix
  Rodrigues(rvec,rvecR);
  
  vector<Point2f> rectanglePoints_i;
  
  projectPoints(rectanglePoints_v, rvecR, T, K, distortionVector, rectanglePoints_i);
  
  cout << " Projecting Point ... " << endl;
  cout << " rectanglePoints_i ... " << rectanglePoints_i << endl;
  
  Rect imageBounds(Point(), imgDepth.size());
  
    // Show Rectangle .............
  Size safetyRadius_i; // Dimensions of safety rectangle (NOT Radius)
  safetyRadius_i.width = norm(rectanglePoints_i[1] - rectanglePoints_i[2]);
  safetyRadius_i.height = norm(rectanglePoints_i[1] - rectanglePoints_i[4]);
   
  Rect shield = Rect( rectanglePoints_i[1], safetyRadius_i);
  	// .............................
  
  for (int i = rectanglePoints_i[1].y; i < rectanglePoints_i[4].y; i ++)
  	for (int j = rectanglePoints_i[1].x; j < rectanglePoints_i[2].x; j ++)
  	{
  		//cout << " Try ... " << endl;
  		if(!imageBounds.contains(Point(j,i)))
  		{
  		isInCollision = 2;
  		cout << "Out of Bounds ... " << endl;
  		return isInCollision;
  		}
  		
			if(imgDepth.at<float>(i,j) < rectanglePoints_v[1].z)
			{
			imgRgb.at<float>(i,j) = 255;
			numCollisionPoints += 1;
			}
			
			if(numCollisionPoints == 5)
			{
			// Show Rectangle .............
			//rectangle(imgRgb, shield, Scalar(255,255,255), 0.5);
 			//circle(imgRgb, rectanglePoints_i[0], 0.5, Scalar(0,0,0), 0.5);
 			// .............................
 			cout << imgDepth.at<float>(i,j) << " compared with " << rectanglePoints_v[1].z << endl;
 			// .............................
  
			isInCollision = 1;
			cout << "Collision Detected ... " << endl;
			return isInCollision;
			}
			
			// Show Rectangle .............
			//rectangle(imgRgb, shield, Scalar(0,0,0), 0.5);
 			//circle(imgRgb, rectanglePoints_i[0], 0.5, Scalar(255,255,255), 0.5);
 			// .............................
  	}
  	
  cout << " Projecting Point ... " << endl;
  
  // Show Rectangle
  /*
  Size safetyRadius_i; // Dimensions of safety rectangle (NOT Radius)
  safetyRadius_i.width = norm(rectanglePoints_i[1] - rectanglePoints_i[2]);
  safetyRadius_i.height = norm(rectanglePoints_i[1] - rectanglePoints_i[4]);
  
  Rect shield = Rect( rectanglePoints_i[1], safetyRadius_i);
  
  if(isInCollision == 1)
  {
  rectangle(imgDepth, shield, Scalar(255,255,255), 5);
  circle(imgDepth, rectanglePoints_i[0], 10, Scalar(0,0,0), 5);
  }
  else
  {
  rectangle(imgDepth, shield, Scalar(0,0,0), 5);
  circle(imgDepth, rectanglePoints_i[0], 10, Scalar(255,255,255), 5);
  }
  
  */
   
return isInCollision;
}
