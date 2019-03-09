#pragma once
#include <opencv2/opencv.hpp>
//angThresh,threshold for rotation about x and y axis
//allRot = false , only using the rotation with z axis
bool wrapImagePerspective(cv::Mat &src, cv::Mat &dst, cv::Mat &H, double angThresh = 10, bool bShape = true, bool allRot = true, bool bDrawLis = true);

void calImageRotation(cv::Mat &src, cv::Mat &R, double angThresh = 10, bool allRot = true);

void wrapImagePerspectiveH(cv::Mat &src, cv::Mat &dst, cv::Mat &H);

void wrapImagePerspectiveR(cv::Mat &src, cv::Mat &dst, cv::Mat R);
