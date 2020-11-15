/* Code from https://www.e-consystems.com/Articles/Camera/opencv-jetson-using-13MP-MIPI-camera.asp */

#include <chrono>
#include <ctime>
#include <stdio.h>
#include "opencv2/opencv.hpp"
using namespace cv;
Mat frame;

int main(int, char**)
{
std::chrono::steady_clock::time_point start;
    std::chrono::system_clock::time_point now;
    std::chrono::steady_clock::time_point end;

	VideoCapture cap(1); // open the default camera - this seems to be slower than using v4l2
    //VideoCapture cap("v4l2src device=\"/dev/video0\" name=e ! video/x-raw, width=1920, height=1080, format=(string)BGR ! videoconvert ! appsink"); // open the default camera
	if(!cap.isOpened())  // check if we succeeded
		return -1;
	cap.set(CAP_PROP_FRAME_WIDTH,1920);
	cap.set(CAP_PROP_FRAME_HEIGHT,1080);

    double fps = cap.get(CAP_PROP_FPS);
    // If you do not care about backward compatibility
    // You can use the following instead for OpenCV 3
    // double fps = video.get(CAP_PROP_FPS);
    printf("Frames per second using video.get(CV_CAP_PROP_FPS) : %f\n", fps);

	Mat edges;
	namedWindow("edges", WINDOW_AUTOSIZE);
	for(;;)
	{
		start = std::chrono::steady_clock::now();
		cap >> frame; // get a new frame from camera
		if (frame.empty())
			break;
		cvtColor(frame, edges, COLOR_BGR2GRAY);
		GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
		Canny(edges, edges, 0, 30, 3);
		//imshow("edges", edges);
		//if((char)waitKey(1) == 27) break;
		end = std::chrono::steady_clock::now();
		printf("FPS: %2.2f\n", 1000000.f/std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}
