// Takes a set of points in world coordinate frame, transform them and check each of them for collision using cspace2collision 

#include "local_functions.h"

using namespace std;
using namespace cv;
using namespace Eigen;


vector<int> waypoints2collision
(vector<Point3f> queryPoints, vector<float> currentPose, float safetyRadius, Mat& projectionMatrix, Mat& distortionVector, Mat& imgDepth, Mat& imgRgb)
{

vector<int> areInCollision;

Vector3f queryPoint;

Vector3f currentPosition(currentPose[0], currentPose[1], currentPose[2]);
Quaternionf currentOrientation(currentPose[6], currentPose[3], currentPose[4], currentPose[5]);

Vector3f camOffset(0.10,0,0); // camera offset from vehicle origin 

Transform<float,3,Affine> worldToBody(currentOrientation.normalized().inverse() * Translation3f(-1 * currentPosition));
Transform<float,3,Affine> bodyToCam(AngleAxisf(-90*M_PI/180, Vector3f::UnitY()) * AngleAxisf(90*M_PI/180, Vector3f::UnitX()) * Translation3f(-1 * camOffset));

/*
cout << "worldToBody Affine : " << endl;
for (int i = 0; i < 4; i++)
{
 for (int j = 0; j < 4; j++)
	cout << worldToBody(i,j) << " ";
	cout << endl;
}

cout << "bodyToCam Affine : " << endl;
for (int i = 0; i < 4; i++)
{
 for (int j = 0; j < 4; j++)
	cout << bodyToCam(i,j) << " ";
	cout << endl;
}
*/

int i = 0;
areInCollision.clear();
for (vector<Point3f>::iterator it = queryPoints.begin(); it < queryPoints.end(); it++)
{
//queryPoints[i].x = 1.2;
//queryPoints[i].y = 0;
//queryPoints[i].z = 0;

//        projectionMatrix.at<double>(0,0) = 205.46;
//        projectionMatrix.at<double>(0,2) = 320.5;
//        projectionMatrix.at<double>(1,1) = 205.46;
//        projectionMatrix.at<double>(1,2) = 240.5;
 //       projectionMatrix.at<double>(2,2) = 1;

cout << " queryPoint " << i << " : " << queryPoints[i] << endl;

queryPoint(0) = queryPoints[i].x;
queryPoint(1) = queryPoints[i].y;
queryPoint(2) = queryPoints[i].z;

queryPoint = bodyToCam * worldToBody * queryPoint;

cout << "transformedQueryPoint : " << queryPoint << endl;

queryPoints[i].x = queryPoint(0);
queryPoints[i].y = queryPoint(1);
queryPoints[i].z = queryPoint(2);

cout << " Checking Point : " << i << endl;

areInCollision.push_back(cspace2collision(queryPoints[i], safetyRadius, projectionMatrix, distortionVector, imgDepth, imgRgb));

cout << " Checked Point : " << i << ", areInCollision : "<< areInCollision[i] << endl;

//getchar();

i+=1;
}
  
return areInCollision;
}

// *********************************************************************************************************************************************************************// 

Point3_<float> find_escape(bitset<4>& direction, Point3_<float> pointUnderCollision, vector<float> currentPose, float safetyRadius, Mat& projectionMatrix, Mat& distortionVector, Mat& imgDepth, Mat& imgRgb)
{

// Assumes quadrotor starts from (0,0,0)

float dPlane = 2.5 * safetyRadius; // distance between expected collision plane and the projection ahead of the plane in which collision is expected to occur in z image coordinate
float dProjections = 0.5 *safetyRadius; // distance between successive projections in x-y image dimensions

vector<Point3f> projectedPoints;
Point3f projectedPoint; // potential escape point

bool isBlocked[4] = {0,0,0,0};

int i = 1;
while(1)
{

projectedPoints.clear();

// Plane in which Collision is Expected to Occur .. &&& .. Parallel Plane at Some Distance From Plane in which Collision is Expected to Occur

// Towards Right[0]
if(direction.test(0)) // 0 0 0 1
{
//projectedPoint.x = pointUnderCollision.x;
//projectedPoint.y = pointUnderCollision.y - i*dProjections;
//projectedPoint.z = pointUnderCollision.z;

//cout << "Point 0 : " << projectedPoint << endl;
//projectedPoints.push_back(projectedPoint);


// Towards Right[1]
projectedPoint.x = pointUnderCollision.x + dPlane;
projectedPoint.y = pointUnderCollision.y - i*dProjections;
projectedPoint.z = pointUnderCollision.z;

projectedPoints.push_back(projectedPoint);
}

// Towards Left[2]
if(direction.test(1)) // 0 0 1 0
{
//projectedPoint.x = pointUnderCollision.x;
//projectedPoint.y = pointUnderCollision.y + i*dProjections;
//projectedPoint.z = pointUnderCollision.z;

//projectedPoints.push_back(projectedPoint);

// Towards Left[3]
projectedPoint.x = pointUnderCollision.x + dPlane;
projectedPoint.y = pointUnderCollision.y + i*dProjections;
projectedPoint.z = pointUnderCollision.z;

projectedPoints.push_back(projectedPoint);
}

// Upwards[4]
if(direction.test(2)) // 0 1 0 0
{
//projectedPoint.x = pointUnderCollision.x;
//projectedPoint.y = pointUnderCollision.y;
//projectedPoint.z = pointUnderCollision.z + i*dProjections;

//projectedPoints.push_back(projectedPoint);

// Upwards[5]
projectedPoint.x = pointUnderCollision.x + dPlane;
projectedPoint.y = pointUnderCollision.y;
projectedPoint.z = pointUnderCollision.z + i*dProjections;

projectedPoints.push_back(projectedPoint);
}

// Downwards[6]
if(direction.test(3)) // 1 0 0 0
{
//projectedPoint.x = pointUnderCollision.x;
//projectedPoint.y = pointUnderCollision.y;
//projectedPoint.z = pointUnderCollision.z - i*dProjections;

//projectedPoints.push_back(projectedPoint);

// Downwards[7]
projectedPoint.x = pointUnderCollision.x + dPlane;
projectedPoint.y = pointUnderCollision.y;
projectedPoint.z = pointUnderCollision.z - i*dProjections;

projectedPoints.push_back(projectedPoint);
}

vector<int> areInCollision = waypoints2collision(projectedPoints, currentPose, safetyRadius, projectionMatrix, distortionVector, imgDepth, imgRgb);

cout << "Checked Points : " << endl;
for (int k = 0 ; k < direction.count() ; k++)
{
cout<< projectedPoints[k] <<  " : " << areInCollision[k] << endl;
}
//getchar();

int z = 0;
for(int k = 0; k < 4; k++)
{
	if(direction.test(k) && areInCollision[z] == 0)
	{
	direction.reset();
	direction.set(k);
	return projectedPoints[z] - Point3_<float>(dPlane, 0, 0);
	}
	
	if(direction.test(k) && areInCollision[z] == 1)
	z += 1;
	
	if(direction.test(k) && areInCollision[z] == 2)
	return Point3_<float>(-1,-1,-1);
}

/*

if(areInCollision[0] == 0 && areInCollision[1] == 0)
return projectedPoints[0];

if(areInCollision[2] == 0 && areInCollision[3] == 0)
return projectedPoints[2];

if(areInCollision[4] == 0 && areInCollision[5] == 0)
return projectedPoints[4];

if(areInCollision[6] == 0 && areInCollision[7] == 0)
return projectedPoints[6];

if(areInCollision[0] == 2)
isBlocked[0] = 1;

if(areInCollision[2] == 2)
isBlocked[1] = 1;

if(areInCollision[4] == 2)
isBlocked[2] = 1;

if(areInCollision[6] == 2)
isBlocked[3] = 1;

if(isBlocked[0] && isBlocked[1] && isBlocked[2] && isBlocked[3])
return Point3_<float>(-1,-1,-1);
*/
i += 1;
}

}
