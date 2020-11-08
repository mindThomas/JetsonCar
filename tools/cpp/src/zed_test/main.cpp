#include <stdio.h>
#include <iostream>
#include <string>
#include <chrono> // for sleep
#include <thread>
#include <signal.h>

#include "zed/zed.h"
#include <opencv2/opencv.hpp>

void PoseCallback(ZED::Pose &pose);
void ImageCallback(cv::Mat &image);

bool ShouldExit = false;
void exitHandler(int s) {
    std::cout << "Should exit" << std::endl;
    ShouldExit = true;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, exitHandler);
    std::cout << "ZED Test" << std::endl;

    ZED zed;

    zed.Connect();

    zed.RegisterCallback_Pose(PoseCallback);
    //zed.RegisterCallback_LeftImage(&ImageCallback, true);

    cv::namedWindow("Color");
    cv::namedWindow("Depth");

    while (!ShouldExit)
    {
        std::cout << std::flush;

        //std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto left = zed.GetLeftImage();
        auto right = zed.GetRightImage();
        cv::Mat combined, combined_downscaled;
        cv::hconcat(left, right, combined);
        cv::resize(combined, combined_downscaled, cv::Size(), 0.5, 0.5);
        cv::imshow("Color", combined_downscaled);

        auto depth = zed.GetDepthImage();
        if (depth.rows > 0)
            cv::imshow("Depth", depth);

        cv::waitKey(20);
    }

    zed.Disconnect();
}

void PoseCallback(ZED::Pose &pose)
{
    auto XYZ = ZED::GetPoseTranslation(pose.pose);
    auto RPY = ZED::GetPoseEulerRPY(pose.pose, true);
    printf("XYZ = %2.2f, %2.2f, %2.2f\n", XYZ(0), XYZ(1), XYZ(2));
    printf("RPY = %2.2f, %2.2f, %2.2f\n\n", RPY(0), RPY(1), RPY(2));
}

void ImageCallback(cv::Mat &image)
{
    cv::namedWindow("Left");
    cv::imshow("Left", image);
    cv::waitKey(1);
}