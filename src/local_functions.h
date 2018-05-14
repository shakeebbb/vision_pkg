
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry" 
#include "iostream"
#include "fstream"
#include "ctime"
#include "bitset"  

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"

int cspace2collision(cv::Point3f, float, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);

std::vector<cv::Point3f> waypoints_gen(std::vector<float>&, std::vector<float>, int&, cv::Point3_<float>&, float, float, float, std::bitset<4>&, bool&, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);

std::vector<int> waypoints2collision(std::vector<cv::Point3f>, std::vector<float>, float, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);

std::vector<cv::Point3f> lqr_solver(Eigen::MatrixXf, Eigen::MatrixXf, float, float, float, Eigen::MatrixXf, std::vector<float>&);

cv::Point3_<float> find_escape(std::bitset<4>&, cv::Point3_<float>, std::vector<float>, float, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);
