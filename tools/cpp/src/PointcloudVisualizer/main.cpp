/* Copyright (C) 2018-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include <signal.h>
#include <algorithm>
#include <condition_variable>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/format.hpp>
#include <boost/filesystem/convenience.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

#include <jetsoncar_utils.h>

#include <GlVisualizer.h>
#include <pcl_point_types.h>

#define deg2rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2deg(angleRadians) ((angleRadians) * 180.0 / M_PI)

using Eigen::MatrixXd;
bool ShouldExit = false;

void exitHandler(int s) {
    std::cout << "Should exit" << std::endl;
    ShouldExit = true;
}

int main(int argc, char** argv ) {
    std::string argv_str(realpath(argv[0], 0));
    std::string base = argv_str.substr(0, argv_str.find_last_of("/"));

    std::string FilePath = "";
    if (argv[1]) {
        if (!std::string(argv[1]).compare("-h")) {
            std::cout << "Examples of usecases:" << std::endl;
            std::cout << " ./PointcloudVisualization <file>" << std::endl;
            std::cout << "    Single file visualization (if supported)" << std::endl;
            exit(0);
        }

        FilePath = std::string(argv[1]);
        if (!utils::FileExist(std::string(argv[1]))) {
            printf("Error: File does not exist!\n");
            exit(-1);
        }

        if (utils::getFileExtension(FilePath).compare(".pcd")) {
            printf("Error: Incorrect file type %s (should be .pcd)\n", utils::getFileExtension(FilePath).c_str());
            exit(-1);
        }
    }
    else {
        printf("Error: File not specified!\n");
        exit(-1);
    }

    printf("Binary source folder: %s\n", __SOURCE_FOLDER__);
    printf("Binary output folder: %s\n", base.c_str());

    std::cout << "Opening visualizer" << std::endl;

    GlVisualizer visualizer(false);
    signal(SIGINT, exitHandler);
    visualizer.RegisterExitHandler(exitHandler);

    pcl::PointCloud<PointTypes::LidarPoint>::Ptr pc(new pcl::PointCloud<PointTypes::LidarPoint>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(FilePath, *pc);

    for (auto pt : pc->points) {
        pcl::PointXYZRGB pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        pt2.z = pt.z;

        pt2.r = 0xFF;
        pt2.g = 0xFF;
        pt2.b = 0xFF;

        pc2->push_back(pt2);
    }
    visualizer.AddPointcloud("LiDAR");
    visualizer.AddFrame(Eigen::Transform<float,3,Eigen::Affine>::Identity(), "Origin");
    visualizer.UpdatePointcloud("LiDAR", pc2.get());
    visualizer.Display();

    while (!ShouldExit && visualizer.isRunning()) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }

    if (visualizer.isRunning()) {
        visualizer.Exit();
    }

    std::cout << "Terminating main program" << std::endl;

    return 0;
}
