/* blockDetect.cpp
2017/11/23
the test example for detect the color block.
*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include "BlockDetect.h"
//#include <Windows.h>
using namespace std;
using namespace cv;

#define BLOCK "block"

Mat srcImage, _srcImage, channelsBGR[3];
Mat srcImageHSV, channelsHSV[3];
Mat dstImageGray;

Mat srcImage1;


COLOR color = BLUE;
const string Color[5] = {"BLUE", "GREEN", "RED", "YELLOW","BLACK"};
BlockData blockData, blockData2;

int main()
{
	for (;;)
	{
		srcImage = getFrame(0);
		//srcImage1 = getFrame(2);

		//srcImage = pespectiveTrans(srcImage);
		//resize(srcImage, srcImage, Size(640, 480));
		//imshow(BLOCK, srcImage);

		blockData = getBlockData(srcImage, color, true, "result1");
		//blockData2 = getBlockData(srcImage1, color, true, "result2");
		//cout << "block data:\n";
		//cout << "number = " << blockData.flag << endl;
		//cout << "color = " << Color[blockData.color] << endl;
		//cout << "centerX = " << blockData.centerX << endl;
		//cout << "centerY = " << blockData.centerY << endl;
		//cout << "edge length = " << blockData.edgeLength << endl;
		waitKey(30);
	}
	return 0;
}