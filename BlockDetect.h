#ifndef BLOCKDETECT_H
#define BLOCKDETECT_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Windows.h>
//#include <math.h>

using std::vector;
using std::string;
using cv::Mat;
using cv::RotatedRect;
using cv::VideoCapture;
using cv::Size;
enum COLOR { BLUE, GREEN, RED, YELLOW, BLACK };

struct BlockData
{
	COLOR color;
	float centerX;
	float centerY;
	float edgeLength;
	int flag;
};

//struct BlockSpeed
//{
//	float centerX1;
//	float centerY1;
//	float centerX2;
//	float centerY2;
//	float tms;
//};

Mat getFrame(int cameraId = 0, Size size = Size(640, 480));

Mat pespectiveTrans(Mat);

Mat colorDetect(Mat, COLOR);

vector<RotatedRect> blockDetect(Mat);

float computeAreaRatio(Mat, RotatedRect);

vector<RotatedRect> selectBlock(vector<RotatedRect>);

void drawRect(Mat, vector<RotatedRect>, string windowName);

BlockData getBlockData(Mat, COLOR, bool drawResult = true, string windowName = "Result");

#endif // !BLOCKDETECT_H

