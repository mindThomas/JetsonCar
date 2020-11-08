#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono> // for sleep
#include <thread>
#include <signal.h>

#include "feature_extractor.h"
#include <opencv2/opencv.hpp>

bool ShouldExit = false;
void exitHandler(int s) {
    std::cout << "Should exit" << std::endl;
    ShouldExit = true;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, exitHandler);    
    std::cout << "Feature Extractor Testbench" << std::endl;

    cv::Mat image = cv::imread(__SOURCE_FOLDER__"/apriltag-36h11.jpg");
    feature_extractor extractor;
    extractor.BRISK_Example(image);

    /*while (!ShouldExit)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }*/
}
