#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**)
{
    VideoCapture cap("v4l2src device=\"/dev/video1\" name=e ! video/x-raw, width=1920, height=1080, format=(string)BGR ! videoconvert ! appsink");
    if(!cap.isOpened())
        return -1;
    

    namedWindow("Preview",1);
    Mat frame;
    for(;;)
    {
        cap >> frame; // get a new frame from camera
        imshow("Preview", frame);
        if(waitKey(2) >= 0) break;
    }
    return 0;
}
