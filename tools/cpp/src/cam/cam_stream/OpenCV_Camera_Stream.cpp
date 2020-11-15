/* Code from https://www.e-consystems.com/Articles/Camera/opencv-jetson-using-13MP-MIPI-camera.asp */

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

Mat image;
int main()
{
	VideoCapture cap;
	char keypressed;
	cap.open(1);
	if( !cap.isOpened() )
	{
		cout << "***Could not initialize capturing...***\n";
		return -1;
	}
	cap.set(CAP_PROP_FRAME_WIDTH,3840);
	cap.set(CAP_PROP_FRAME_HEIGHT,2160);
	cout << "Width x Height = " << cap.get(CAP_PROP_FRAME_WIDTH) << " x " 
    << cap.get(CAP_PROP_FRAME_HEIGHT) << "\n";
	namedWindow("Camera", WINDOW_AUTOSIZE);
	for(;;)
	{
		cap >> image;
		if( image.empty() )
			break;
		imshow("Camera", image);
		keypressed = (char)waitKey(10);
		if( keypressed == 27 )
			break;
	}
	cap.release();
	return 0;
}
