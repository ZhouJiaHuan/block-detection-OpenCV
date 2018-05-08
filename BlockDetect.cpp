#include "BlockDetect.h"
using namespace std;
using namespace cv;

namespace
{
	VideoCapture capture0(0);
	VideoCapture capture1(1);
	VideoCapture capture2(2);
}

/*
Description: get one frame image from camera.
Arguments:
cameraId - the camera used (default: 0).
size - frame size (default: 640*480).
Return: BGR-format Image with specified size.
*/
Mat getFrame(int cameraId, Size size)
{
	Mat frame;
	while (!frame.data)
	{
		switch (cameraId)
		{
		case 0:
			//capture0 >> frame;
			capture0.read(frame);
			break;
		case 1:
			//capture1 >> frame;
			capture1.read(frame);
			break;
		case 2:
			//capture2 >> frame;
			capture2.read(frame);
			break;
		default:
			break;
		}
	}
	resize(frame, frame, size);
	return frame;
}


Mat pespectiveTrans(Mat frame)
{
	int imgWidth = frame.cols;
	int imgHeight = frame.rows;
	vector<Point2f> corners(4);
	vector<Point2f> cornersT(4);

	corners[0] = Point2f(114, 36);
	corners[1] = Point2f(535, 42);
	corners[2] = Point2f(114, 454);
	corners[3] = Point2f(530, 452);

	cornersT[0] = Point2f(114, 37);
	cornersT[1] = Point2f(532, 37);
	cornersT[2] = Point2f(114, 455);
	cornersT[3] = Point2f(532, 455);

	Mat transform = getPerspectiveTransform(corners, cornersT);
	warpPerspective(frame, frame, transform, frame.size(), INTER_LINEAR);
	return frame;
}


/*
Description: split the specified color from BGR image provided.
Arguments:
srcImageBGR - source BGR-format image.
color - an enum type specifying the color amoung BLUE, GREEN, RED and YELLOW.
Return: an binary Image.
*/
Mat colorDetect(Mat srcImageBGR, COLOR color)
{
	Mat _srcImageBGR = srcImageBGR.clone();
	Mat srcImageHSV, channelsHSV[3];
	Mat dstImageGray;
	//resize(_srcImageBGR, _srcImageBGR, Size(640, 480));
	cvtColor(_srcImageBGR, srcImageHSV, CV_BGR2HSV);
	split(srcImageHSV, channelsHSV);

	int h, s, v;
	int H, S, V;
	int h1 = 156, H1 = 180;
	switch (color)
	{
	case BLUE:
		h = 100; H = 124;
		s = 43; S = 255;
		v = 46; V = 255;
		break;
	case GREEN:
		h = 35; H = 77;
		s = 120; S = 255;
		v = 80; V = 255;
		break;
	case RED:
		h = 0; H = 10;
		s = 43; S = 255;
		v = 46; V = 255;
		break;
	case YELLOW:
		h = 23; H = 36;
		s = 43; S = 255;
		v = 46; V = 255;
		break;
	case BLACK:
		h = 0; H = 180;
		s = 0; S = 255;
		v = 0; V = 80;
	default:
		break;
	}
	inRange(srcImageHSV, Scalar(h, s, v), Scalar(H, S, V), dstImageGray);
	//if (color == RED)
	//{
	//	Mat dstImageGray1;
	//	inRange(srcImageHSV, Scalar(h1, s, v), Scalar(H1, S, V), dstImageGray1);
	//	dstImageGray = dstImageGray + dstImageGray1;
	//} 
	//blur(dstImageGray, dstImageGray, Size(9, 9));
	
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	erode(dstImageGray, dstImageGray, element);
	erode(dstImageGray, dstImageGray, element);
	//erode(dstImageGray, dstImageGray, element);
	dilate(dstImageGray, dstImageGray, element);
	//GaussianBlur(dstImageGray, dstImageGray, Size(9, 9), 0);
	return dstImageGray;
}


/*
Description: compute the contour area ratio in the center of rotate rect.
Arguments:
imgBin - binary image.
rotateRect - rotate rectangle to surround the block
Return: contour area ratio.
*/
float computeAreaRatio(Mat imgBin, RotatedRect rotateRect)
{
	Point2f recPoint[4];
	Point centerPoint;
	rotateRect.points(recPoint);
	float countValue = 0;
	int rectWidth = (int)(sqrt(pow((recPoint[0].x - recPoint[1].x), 2) + pow((recPoint[0].y - recPoint[1].y), 2)));
	int rectHeight = (int)(sqrt(pow((recPoint[1].x - recPoint[2].x), 2) + pow((recPoint[1].y - recPoint[2].y), 2)));
	centerPoint.x = (recPoint[0].x + recPoint[2].x) / 2;
	centerPoint.y = (recPoint[0].y + recPoint[2].y) / 2;
	int initX = (int)(centerPoint.x - rectWidth / 8.0);
	int initY = (int)(centerPoint.y - rectHeight / 8.0);
	initX = initX >= 0 ? initX : 0;
	initY = initY >= 0 ? initY : 0;
	int endX = initX + rectWidth/4.0;
	int endY = initY + rectHeight/4.0;
	endX = endX < imgBin.cols ? endX : imgBin.cols;
	endY = endY < imgBin.rows ? endY : imgBin.rows;

	for (int j = initX; j < endX; j++)
	{
		for (int i = initY; i < endY; i++)
		{
			float binValue = imgBin.at<unsigned char>(i, j);
			if (binValue == 1)
			{
				countValue++;
			}
		}
	}
	return countValue / (rectWidth*rectHeight / 16.0);
}

/*
Description: detect the block from an binary image provided.
Arguments:
srcImageBGR - binary image.
Return: Rotate rects covering all blocks 
*/
vector<RotatedRect> blockDetect(Mat dstImageGray)
{
	//vector<BlockData> blocksData;
	//BlockData blockData;
	vector<vector<Point>> contours;
	vector<Vec4i> hieracy;
	findContours(dstImageGray, contours, hieracy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	vector<vector<Point>> contours_poly(contours.size());
	vector<RotatedRect> rotateRects;

	for (size_t i = 0; i < contours.size(); i++)
	{
		float contLength = arcLength(contours[i], true);
		float contArea = contourArea(contours[i]);
		
		if ((contLength > 150) && (contArea > 1000)) // ´ýµ÷½Ú
		{
			RotatedRect rect;
			Point2f recPoint[4];
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
			rect = minAreaRect(Mat(contours_poly[i]));
			rect.points(recPoint);

			float rectWidth, rectHeight, H_W, W_H, areaRatio;
			rectWidth = sqrt(pow((recPoint[0].x - recPoint[1].x), 2) + pow((recPoint[0].y - recPoint[1].y), 2));
			rectHeight = sqrt(pow((recPoint[1].x - recPoint[2].x), 2) + pow((recPoint[1].y - recPoint[2].y), 2));
			H_W = abs(rectHeight / rectWidth);
			W_H = abs(rectWidth / rectHeight);
			areaRatio = computeAreaRatio(dstImageGray, rect);

			if ((H_W<1.3) && (W_H<1.3) && (areaRatio>0.6))
			{
				cout << "conLength = " << contLength << endl;
				cout << "area = " << contArea << endl;
				float ratio = H_W > W_H ? H_W : W_H;
				rotateRects.push_back(rect);
			}
		}
	}
	return rotateRects;
}


/*
Description: select one block from all blocks detected.
Arguments:
rotateRects - rotateRects saving all blocks data.
Return: one block selected.
*/
vector<RotatedRect> selectBlock(vector<RotatedRect> rotateRects)
{
	if (rotateRects.size() <= 1)
	{
		return rotateRects;
	}

	vector<RotatedRect> rotateRect;
	float centerX, centerY;
	float rectWidth, rectHeight;
	//float minAbs = 50, currentAbs;
	float maxDisToCenter = 1000, currentDis;
	float maxEdge = 500, currentMaxEdge;
	float maxCenterX = 0, maxCenterY = 0;
	for (size_t i = 0; i < rotateRects.size(); i++)
	{
		RotatedRect rect = rotateRects[i];
		Point2f recPoint[4];
		rect.points(recPoint);

		if (recPoint[0].x<0 || recPoint[0].y<0 || recPoint[1].x>640 || recPoint[1].y<0 ||
			recPoint[2].x>640 || recPoint[0].y>480 || recPoint[3].x<0 || recPoint[3].y>480)
		{
			continue;
		}

		centerX = (recPoint[0].x + recPoint[2].x) / 2;
		centerY = (recPoint[0].y + recPoint[2].y) / 2;
		rectWidth = sqrt(pow((recPoint[0].x - recPoint[1].x), 2) + pow((recPoint[0].y - recPoint[1].y), 2));
		rectHeight = sqrt(pow((recPoint[1].x - recPoint[2].x), 2) + pow((recPoint[1].y - recPoint[2].y), 2));
		//currentAbs = abs(rectWidth - rectHeight);
		//currentMaxEdge = rectWidth>rectHeight ? rectWidth : rectHeight;
		//currentDis = sqrt(pow(centerX - 320, 2) + pow(centerY - 240, 2));


		if ((centerX > maxCenterX) && (centerY>maxCenterY))
		{
			rotateRect.clear();
			rotateRect.push_back(rect);
			maxCenterX = centerX;
			maxCenterY = centerY;
			//maxEdge = currentMaxEdge;
		}
	}
	return rotateRect;
}


/*
Description: draw rotate rectangles surrounding the blocks
Arguments:
srcImage - the source image where the rectangles are drawn on.
rotateRects - rectangles to be drawn.
Return: None
*/
void drawRect(Mat srcImage, vector<RotatedRect> rotateRects, string windowName)
{
	Mat _srcImage = srcImage.clone();
	Point2f rectPoint[4];
	Point centerPoint;
	RNG rng(12345);
	for (size_t i = 0; i < rotateRects.size(); i++)
	{
		Scalar drawColor = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		rotateRects[i].points(rectPoint);
		for (int i = 0; i < 4; i++)
		{
			line(_srcImage, rectPoint[i], rectPoint[(i + 1)%4], drawColor, 4);
		}
		centerPoint.x = (rectPoint[0].x + rectPoint[2].x) / 2;
		centerPoint.y = (rectPoint[0].y + rectPoint[2].y) / 2;
		circle(_srcImage, centerPoint, 4, drawColor, -1, 8, 0);
	}
	imshow(windowName, _srcImage);
}


/*
Description: get block data with specified color.
Arguments:
srcImage - source BGR-format image.
color - an enum type specifying the color amoung BLUE, GREEN, RED and YELLOW.
drawResult - draw the rectangle surrounding the block (default: true).
windowName - window name of the result (default: "Result").
Return: struct type used for saving the block data.
*/
BlockData getBlockData(Mat srcImage, COLOR color, bool drawResult, string windowName)
{
	BlockData blockData;
	blockData.centerX = 0;
	blockData.centerY = 0;
	blockData.color = color;
	blockData.edgeLength = 0;
	blockData.flag = 0;

	Mat dstImageGray = colorDetect(srcImage, color);
	imshow("Gray Image", dstImageGray);

	vector<RotatedRect> rotateRects = blockDetect(dstImageGray);
	vector<RotatedRect> rotateRect = selectBlock(rotateRects);
	Point2f rectPoint[4];
	
	if (rotateRect.size() == 1)
	{
		float rectWidth, rectHeight;
		rotateRect[0].points(rectPoint);
		rectWidth = sqrt(pow((rectPoint[0].x - rectPoint[1].x), 2) 
			+ pow((rectPoint[0].y - rectPoint[1].y), 2));
		rectHeight = sqrt(pow((rectPoint[1].x - rectPoint[2].x), 2) 
			+ pow((rectPoint[1].y - rectPoint[2].y), 2));
		blockData.edgeLength = rectWidth < rectHeight ? rectWidth : rectHeight;
		blockData.centerX = (rectPoint[0].x + rectPoint[2].x) / 2;
		blockData.centerY = (rectPoint[0].y + rectPoint[2].y) / 2;
		blockData.flag = 1;
	}

	if (drawResult)
	{
		drawRect(srcImage, rotateRects, windowName);
	}
	return blockData;
}