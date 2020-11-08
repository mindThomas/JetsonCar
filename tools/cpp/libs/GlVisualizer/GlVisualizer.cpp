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

#include "GlVisualizer.h"

#include <stdio.h>
#include <string>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>

/* Visualize PCD file */
GlVisualizer::GlVisualizer(const std::string &screenName, bool slidersAndButtonsEnabled) :
    screenName_(screenName),
    guiIsReady_(false),
    toolboxEnabled_(slidersAndButtonsEnabled),
    glFont(__SOURCE_FOLDER__"/Lato-Bold.ttf", AXIS_FONT_SIZE),
    shaderProgram(0),
    Viewport(pangolin::ModelViewLookAt(-10, -10, 5, 0, 0, 0, pangolin::AxisZ)),
    shouldExit(false),
    isRunning_(false),
    ExitHandler_(NULL)
{
    VisualizerThread.reset(new boost::thread(boost::bind(&GlVisualizer::Visualize, this)));
}

GlVisualizer::GlVisualizer(void) : GlVisualizer("Default", false) {}
GlVisualizer::GlVisualizer(const std::string &screenName) : GlVisualizer(screenName, false) {}
GlVisualizer::GlVisualizer(char * screenName) : GlVisualizer(std::string(screenName), false) {}
GlVisualizer::GlVisualizer(bool slidersAndButtonsEnabled) : GlVisualizer("Default", slidersAndButtonsEnabled) {}

void GlVisualizer::Visualize(void)
{
    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind(screenName_,640,480);
    //pangolin::SetFullscreen(true);
    glewInit();

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    s_cam.reset(new pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1,
                                       5000), // w, h, fu, fv, u0, v0, zNear, zFar
//					pangolin::ModelViewLookAt(4, -3, 2, 1, 1, 1, pangolin::AxisZ))); // the camera starts at (4,-3,2) looking at point (1,1,1) -- z axis is up
            Viewport));

    defaultViewpoint = GetViewpoint();

    // Aspect ratio allows us to constrain width and height whilst fitting within specified
    // bounds. A positive aspect ratio makes a view 'shrink to fit' (introducing empty bars),
    // whilst a negative ratio makes the view 'grow to fit' (cropping the view).
    sidebarWidth_ = 0;
    if (toolboxEnabled_)
        sidebarWidth_ = 180;

    //pangolin::GlHandler3D handler(*s_cam);
    handler.reset(new pangolin::GlHandler3D(*s_cam));
    pangolin::View &d_cam = pangolin::Display("cam")
            .SetBounds(pangolin::Attach(0.0), pangolin::Attach(1.0), pangolin::Attach::Pix(sidebarWidth_), pangolin::Attach(1.0), -640.f / 480.f)
            .SetHandler(handler.get());

    if (toolboxEnabled_)
        pangolin::CreatePanel("ui").SetBounds(pangolin::Attach(0.0), pangolin::Attach(1.0), pangolin::Attach(0.0), pangolin::Attach::Pix(sidebarWidth_));

    size_t imagesPrepared = 0;
    while (!shouldExit && (guiIsReady_ == false || imagesPrepared != Images.size()))
    {
        for (auto& image : Images) {
            if (image.view == 0) {
                pangolin::View &view = pangolin::Display(image.name);
                image.view = &view;

                double aspect = float(image.width) / float(image.height);
                switch (image.dock) {
                    case TopLeft:
                        if (image.widthPct > 0)
                            image.view->SetBounds(pangolin::Attach(1.0-image.widthPct/aspect), pangolin::Attach(1.0), pangolin::Attach(0.0), pangolin::Attach(image.widthPct), aspect);
                        else
                            image.view->SetBounds(pangolin::Attach(1.0), pangolin::Attach::ReversePix(image.height), pangolin::Attach::Pix(sidebarWidth_), pangolin::Attach::Pix(sidebarWidth_+image.width), aspect);
                        image.view->SetLock(pangolin::LockLeft, pangolin::LockTop);
                        break;

                    case TopCenter:
                        if (image.widthPct <= 0)
                            image.widthPct = 0.25;
                        image.view->SetBounds(pangolin::Attach(1.0-image.widthPct/aspect), pangolin::Attach(1.0), pangolin::Attach(0.5-image.widthPct/2), pangolin::Attach(0.5+image.widthPct/2), aspect);
                        image.view->SetLock(pangolin::LockCenter, pangolin::LockTop);
                        break;

                    case TopRight:
                    default:
                        if (image.widthPct > 0)
                            image.view->SetBounds(pangolin::Attach(1.0-image.widthPct/aspect), pangolin::Attach(1.0), pangolin::Attach(1.0-image.widthPct), pangolin::Attach(1.0), aspect);
                        else
                            image.view->SetBounds(pangolin::Attach(1.0), pangolin::Attach::ReversePix(image.height), pangolin::Attach::ReversePix(image.width), pangolin::Attach(1.0), aspect);
                        image.view->SetLock(pangolin::LockRight, pangolin::LockTop);
                        break;

                    case MidLeft:
                        image.view->SetLock(pangolin::LockLeft, pangolin::LockCenter);
                        break;
                    case MidCenter:
                        image.view->SetLock(pangolin::LockCenter, pangolin::LockCenter);
                        break;
                    case MidRight:
                        image.view->SetLock(pangolin::LockRight, pangolin::LockCenter);
                        break;
                    case BottomLeft:
                        image.view->SetLock(pangolin::LockLeft, pangolin::LockBottom);
                        break;
                    case BottomCenter:
                        image.view->SetLock(pangolin::LockCenter, pangolin::LockBottom);
                        break;
                    case BottomRight:
                        image.view->SetLock(pangolin::LockRight, pangolin::LockBottom);
                        break;

                    case Top:
                        image.view->SetBounds(pangolin::Attach::ReversePix(image.height), pangolin::Attach(1.0), pangolin::Attach::Pix(sidebarWidth_), pangolin::Attach(1.0), aspect);
                        image.view->SetLock(pangolin::LockLeft, pangolin::LockTop);
                        break;

                    case Bottom:
                        image.view->SetBounds(pangolin::Attach(0.0), pangolin::Attach::Pix(image.height), pangolin::Attach::Pix(sidebarWidth_), pangolin::Attach(1.0), aspect);
                        image.view->SetLock(pangolin::LockLeft, pangolin::LockBottom);
                        break;
                }

                image.texture = new pangolin::GlTexture(image.width,image.height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
                //image.data = new unsigned char[3*image.width*image.height];
                //memset(image.data, 0, 3*image.width*image.height);
                imagesPrepared++;
            }
        }

        boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const char * vertexShader = "#version 410 compatibility\n"
            "out vec4 colorV;\n"
            " \n"
            "void main() \n"
            "{\n"
            "    vec4 color = gl_Color;\n"
            "    vec4 position = gl_Vertex;\n"
            "    colorV = color;\n"
            //"    colorV = vec4(max(position.z/3,0) + 0.1, max(position.z/3,0) + 0.1, max(position.z/3,0) + 0.1, 1.0);\n"
            "    gl_Position = gl_ModelViewProjectionMatrix * position ;\n"
            "}";
    const char * fragmentShader = "#version 410 compatibility\n"
            "in vec4 colorV;\n"
            "  \n"
            "void main()\n"
            "{\n"
            "    float fragZ = gl_FragCoord.z;\n"
            "    float scale = 1-exp(-10*(1-fragZ));\n"
            "    gl_FragColor = colorV;\n"
            //"    gl_FragColor = vec4(scale*colorV.x, scale*colorV.y, scale*colorV.z, 1);\n"
            //"    gl_FragColor = vec4(scale, scale, scale, 1);\n"
            "}";
    shaderProgram = ConfigureShaders(vertexShader, fragmentShader);

    Eigen::Vector3d highlightedPointPos = handler->getMouse3Dposition();

    // Default hooks for exiting (Esc) and fullscreen (tab).
    printf("\nPress Esc to exit or Tab for fullscreen\n");
    isRunning_ = true;
    while(!pangolin::ShouldQuit() && !shouldExit)
    {
        frame_mtx.lock();

        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Activate (to be able to modify) camera area (3D world)
        d_cam.Activate(*s_cam); glColor3f(1.0,1.0,1.0); // default color

        // Ensure that blending is enabled for rendering text.
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        keyState = handler->KeyState();

        for (auto const& frame : Frames) {
            DrawFrame(frame.transform, frame.name, frame.axes_length);
        }

        /*if (keyState == pangolin::KeyModifierCtrl) {
            highlightedPointPos = handler->getMouse3Dposition();
            //if ((handler.Selected_P_w() - Pick_w).norm() > 1E-6) {
            //    Pick_w = handler.Selected_P_w();
            //    std::cout << pangolin::FormatString("\"Translation\": [%,%,%]", Pick_w[0], Pick_w[1], Pick_w[2])
            //              << std::endl;
            //}

            Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
            tf.block<3,1>(0,3) = highlightedPointPos.cast<float>();
            DrawFrame(tf, "", FRAME_AXIS_LENGTH);
        }*/

        for (auto const& trajectory : Trajectories) {
            for (unsigned int i = 0; i < trajectory.poses.size(); i++) {
                if (i == trajectory.poses.size()-1) // end frame
                    DrawFrame(trajectory.poses[i], trajectory.name, 2*TRAJECTORY_AXIS_LENGTH);
                else
                    DrawFrame(trajectory.poses[i], "", TRAJECTORY_AXIS_LENGTH);
            }
        }

        for (auto& pointcloud : Pointclouds) {
            DrawPointCloud(pointcloud);
            if (pointcloud.eigenvalues.prod() > 0) {
                Eigen::Matrix4f axes = Eigen::Matrix4f::Zero();
                // swap order to have largest eigenvalue as x-axis
                axes.block<3,1>(0,0) = pointcloud.eigenvectors.block<3,1>(0,2);
                axes.block<3,1>(0,1) = pointcloud.eigenvectors.block<3,1>(0,1);
                axes.block<3,1>(0,2) = pointcloud.eigenvectors.block<3,1>(0,0);
                axes.block<4,1>(0,3) = pointcloud.centroid;
                // eigenvalue corresponds to sigma^2 - show axis length as 3*sigma
                DrawFrame(axes, "Cov{" + pointcloud.name + "}", sqrtf(pointcloud.eigenvalues(2))*3, sqrtf(pointcloud.eigenvalues(1))*3, sqrtf(pointcloud.eigenvalues(0))*3);
            }
        }

        while (VBOForClearance.size()) {
            GLuint vbo = VBOForClearance.back();
            if (vbo) glDeleteBuffers(1, &(vbo)); // delete OpenGL buffer allocated to pointcloud
            VBOForClearance.pop_back();
        }

        for (auto& image : Images) {
            if (image.view) {
                // Set some random image data and upload to GPU
                image.texture->Upload(image.image.data,GL_RGB,GL_UNSIGNED_BYTE);

                // Display the image
                image.view->Activate();
                glColor3f(1.0,1.0,1.0);
                image.texture->RenderToViewport();
            }
        }

        frame_mtx.unlock();

        pangolin::FinishFrame(); //pangolin::GetBoundWindow()->SwapBuffers(); pangolin::GetBoundWindow()->ProcessEvents();

        if (toolboxEnabled_) {
            slider_mtx.lock();
            for (auto &slider : Sliders) {
                if (!slider.locked) {
                    if (slider.Double)
                        slider.valueDouble = *slider.pangolinVarDouble; // ->Get()
                    else
                        slider.valueInt = *slider.pangolinVarInt;
                } else {
                    if (slider.Double)
                        *slider.pangolinVarDouble = slider.valueDouble;
                    else
                        *slider.pangolinVarInt = slider.valueInt;
                }
            }
            slider_mtx.unlock();

            checkbox_mtx.lock();
            for (auto &checkbox : Checkboxes) {
                checkbox.value = checkbox.pangolinVar->Get();
            }
            checkbox_mtx.unlock();

            button_mtx.lock();
            for (auto &button : Buttons) {
                if (pangolin::Pushed(*button.pangolin) && !button.handler.empty())
                    button.handler();
            }
            button_mtx.unlock();
        }

        boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    }
    shouldExit = true;

    // Clear up OpenGL allocated buffers before closing
    for (auto& pointcloud : Pointclouds) {
        if (pointcloud.vbo) glDeleteBuffers(1, &(pointcloud.vbo)); // delete OpenGL buffer allocated to pointcloud
        if (pointcloud.vbo_color) glDeleteBuffers(1, &(pointcloud.vbo_color)); // delete OpenGL buffer allocated to pointcloud
    }

    pangolin::DestroyWindow(screenName_);

    printf("\nVisualizer exiting\n");

    isRunning_ = false;

    if (!ExitHandler_.empty())
        ExitHandler_(0);
}

void GlVisualizer::Display(void)
{
    guiIsReady_ = true; // render GUI and lock for control changes
    while (!isRunning_)
        boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // wait for GUI to be ready
}

/*
void GlVisualizer::randomImageData(unsigned char * imageArray, int size)
{
    for(int i = 0 ; i < size;i++) {
        imageArray[i] = (unsigned char)(rand()/(RAND_MAX/255.0));
    }
}
*/

int GlVisualizer::GetKeyState(void)
{
    return keyState;
}

void GlVisualizer::OpenPCD(const std::string & pcd_file)
{
    if (!guiIsReady_) return;
    boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

    printf("Opening Point Cloud example file\n");
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file.c_str(), *pc) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file. \n");
        return;
    }

    AddPointcloud(basename(pcd_file.c_str()), pc);
}

GlVisualizer::pointcloud_t * GlVisualizer::FindPointcloud(const std::string pointcloud_name)
{
    //std::vector<Sensor>::const_iterator it = std::find_if(sensors.begin(), sensors.end(), boost::bind(&Sensor::name, _1) == sensor_name);
    std::vector<GlVisualizer::pointcloud_t>::const_iterator it = std::find_if(Pointclouds.begin(), Pointclouds.end(), find_pointcloud(pointcloud_name));
    if (it != Pointclouds.end())
        return (GlVisualizer::pointcloud_t *)&*it;
    else
        return 0;
}

void GlVisualizer::AddPointcloud(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc, uint8_t size)
{
    AddPointcloud(pointcloud_name, size);
    if (guiIsReady_) // Update Pointcloud can only be called after GlVisualizer::Display() has been called
        UpdatePointcloud(pointcloud_name, new_pc);
}

void GlVisualizer::AddPointcloud(const std::string pointcloud_name, uint8_t size) // add empty pointcloud
{
    boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function

    GlVisualizer::pointcloud_t pointcloud;
    pointcloud.name = pointcloud_name;
    pointcloud.vbo = 0;
    pointcloud.vbo_color = 0;
    pointcloud.vbo_normals = 0;
    pointcloud.changed = false;
    pointcloud.size = size;
    pointcloud.numberOfPoints = 0;
    pointcloud.dense = false;
    pointcloud.visible = true;
    pointcloud.eigenvalues.setZero();
    pointcloud.eigenvectors.setZero();
    Pointclouds.push_back(pointcloud);
}

void GlVisualizer::AddDensePointcloud(const std::string pointcloud_name)
{
    AddPointcloud(pointcloud_name, POINT_SIZE_NORMAL); // white default color with normal size
    GlVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud)
        pointcloud->dense = true;
}

void GlVisualizer::UpdatePointcloud(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB> * new_pc)
{
    //boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    frame_mtx.lock();
    GlVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud) {
        pointcloud->pc = new_pc;
        pointcloud->changed = true;
        frame_mtx.unlock();

        unsigned int cnt = 0;
        while (pointcloud->changed && !shouldExit && cnt++ < 100) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // wait for the pointcloud to be updated
        }
        //pcl::copyPointCloud(*new_pc, *pointcloud->pc);
    } else {
        frame_mtx.unlock();
    }
}

void GlVisualizer::UpdatePointcloudWithNormals(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB> * new_pc, pcl::PointCloud<pcl::Normal> * new_pc_normals)
{
    //boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    frame_mtx.lock();
    GlVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud) {
        pointcloud->pc = new_pc;
        pointcloud->pc_normals = new_pc_normals;
        pointcloud->changed = true;
        frame_mtx.unlock();

        unsigned int cnt = 0;
        while (pointcloud->changed && !shouldExit && cnt++ < 100) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1)); // wait for the pointcloud to be updated
        }
        //pcl::copyPointCloud(*new_pc, *pointcloud->pc);
    } else {
        frame_mtx.unlock();
    }
}

void GlVisualizer::SetPointcloudVisibility(const std::string pointcloud_name, bool visible)
{
    boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function
    GlVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud) {
        pointcloud->visible = visible;
    }
}

void GlVisualizer::UpdatePointcloudWithAxes(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB> * new_pc)
{
    UpdatePointcloud(pointcloud_name, new_pc);

    pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);

    if (pointcloud) {
        pcl::compute3DCentroid<pcl::PointXYZRGB, float>(*new_pc, pointcloud->centroid);
        unsigned int n = pcl::computeCovarianceMatrix<pcl::PointXYZRGB, float>(*new_pc, pointcloud->centroid,
                                                                                  pointcloud->covariance);
        pointcloud->covariance = (1. / (n - 1.)) * pointcloud->covariance;
        pcl::eigen33<Eigen::Matrix<float, 3, 3>, Eigen::Matrix<float, 3, 1>>(pointcloud->covariance,
                                                                             pointcloud->eigenvectors,
                                                                             pointcloud->eigenvalues);
        /*std::cout << "Covariance: " << std::endl << pointcloud->covariance << std::endl;
        std::cout << "Eigenvectors: " << std::endl << pointcloud->eigenvectors << std::endl;
        std::cout << "Eigenvalues: " << std::endl << pointcloud->eigenvalues << std::endl;*/
    }
}

void GlVisualizer::UpdatePointcloud(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc)
{
    UpdatePointcloud(pointcloud_name, new_pc.get());
}

void GlVisualizer::UpdatePointcloudWithNormals(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc, const pcl::PointCloud<pcl::Normal>::Ptr &new_pc_normals)
{
    UpdatePointcloudWithNormals(pointcloud_name, new_pc.get(), new_pc_normals.get());
}

void GlVisualizer::ChangePointSize(const std::string pointcloud_name, uint8_t point_size)
{
    pointcloud_t *pointcloud = FindPointcloud(pointcloud_name);

    if (pointcloud) {
        pointcloud->size = point_size;
    }
}

void GlVisualizer::RemovePointcloud(const std::string pointcloud_name)
{
    frame_mtx.lock();
    GlVisualizer::pointcloud_t * pointcloud = FindPointcloud(pointcloud_name);
    if (pointcloud) {
        if (pointcloud->changed) {
            pointcloud->changed = false;
            boost::this_thread::sleep_for(boost::chrono::milliseconds(10)); // let any locks on "pointcloud.changed" disappear
        }
        VBOForClearance.push_back(pointcloud->vbo);
        VBOForClearance.push_back(pointcloud->vbo_color);

        auto pointcloud_iterator = Pointclouds.begin() + std::distance(Pointclouds.data(), pointcloud); // an equivalent vector::iterator
        Pointclouds.erase(pointcloud_iterator);
    }
    frame_mtx.unlock();
}

void GlVisualizer::ClearPointclouds()
{
    frame_mtx.lock();
    for (auto &pointcloud : Pointclouds) {
        if (pointcloud.changed) {
            pointcloud.changed = false;
            boost::this_thread::sleep_for(boost::chrono::milliseconds(10)); // let any locks on "pointcloud.changed" disappear
        }
        VBOForClearance.push_back(pointcloud.vbo);
        VBOForClearance.push_back(pointcloud.vbo_color);
    }
    Pointclouds.clear();
    frame_mtx.unlock();
}

GlVisualizer::image_t * GlVisualizer::FindImage(const std::string image_name)
{
    //std::vector<Sensor>::const_iterator it = std::find_if(sensors.begin(), sensors.end(), boost::bind(&Sensor::name, _1) == sensor_name);
    std::vector<GlVisualizer::image_t>::const_iterator it = std::find_if(Images.begin(), Images.end(), find_image(image_name));
    if (it != Images.end())
        return (GlVisualizer::image_t *)&*it;
    else
        return 0;
}



void GlVisualizer::AddImage(const std::string image_name, unsigned int width, unsigned int height, DockLocation_t dock, double widthPct) // add empty image
{
    if (guiIsReady_) return; // can only modify displays (add/remove images) while GUI has not been rendered yet
    boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to pc_gl during this function

    GlVisualizer::image_t image;
    image.name = image_name;
    // OBS! It is very important that width and height are divisible by 8
    image.width = 8 * int(round(double(width) / 8));
    image.height = 8 * int(round(double(height) / 8));
    image.dock = dock;
    image.widthPct = widthPct;
    image.image = cv::Mat(image.height, image.width, CV_8UC3, cv::Scalar(0,0,0));
    image.view = 0;
    image.texture = 0;
    image.data = 0;

    Images.push_back(image);
}

void GlVisualizer::UpdateImage(const std::string image_name, cv::Mat &img)
{
    frame_mtx.lock();
    GlVisualizer::image_t * image = FindImage(image_name);
    if (image) {
        if (img.cols == image->width && img.rows == image->height) {
            img.copyTo(image->image);
        } else {
            cv::resize(img, image->image, cv::Size(image->width, image->height), 0, 0, cv::INTER_LINEAR);
        }
        cv::flip(image->image, image->image, 0); // flip vertically
        //cv::flip(image->image, image->image, 1); // flip horizontally
        cv::cvtColor(image->image, image->image, cv::COLOR_BGR2RGB);

        frame_mtx.unlock();
    }
    frame_mtx.unlock();
}

void GlVisualizer::VisualizeNormals(void)
{
    boost::lock_guard<boost::mutex> guard(frame_mtx); // protect the access to normals_gl during this function

    // Compute normals of Point cloud
    printf("Computing normals of Point cloud\n");
    //pcl::PointCloud<pcl::Normal>::Ptr pc_normals(CalculateNormals(pc));

    // Upload normals to GPU
    //normals_gl = UploadNormals(pc, pc_normals, 0.02);
}

Eigen::Matrix<float, 4, 4> GlVisualizer::GetViewpoint(void)
{
    pangolin::OpenGlMatrix tmp = s_cam->GetModelViewMatrix();
    Eigen::Matrix<float, 4, 4> mat = tmp;
    return mat;
}

void GlVisualizer::SetViewpoint(Eigen::Matrix<float, 4, 4> view)
{
    Viewport = pangolin::OpenGlMatrix(view);
    if (s_cam)
        s_cam->SetModelViewMatrix(pangolin::OpenGlMatrix(view));
}

void GlVisualizer::SetDefaultViewpoint()
{
    SetViewpoint(defaultViewpoint);
}

void GlVisualizer::DrawFrame(Eigen::Matrix4f frame, const std::string &frameName, float axis_length)
{
    DrawFrame(frame, frameName, axis_length, axis_length, axis_length);
}

void GlVisualizer::DrawFrame(Eigen::Matrix4f frame, const std::string &frameName, float axis_length_x, float axis_length_y, float axis_length_z)
{
    Eigen::Matrix4f HomogeneusAxis = Eigen::Matrix4f::Zero();
    HomogeneusAxis(0,0) = axis_length_x;
    HomogeneusAxis(1,1) = axis_length_y;
    HomogeneusAxis(2,2) = axis_length_z;
    HomogeneusAxis.block<1,4>(3,0) = Eigen::MatrixXf::Ones(1,4);

    Eigen::Matrix4f axes = frame * HomogeneusAxis;
    const GLfloat vertices[] = { frame(0,3),frame(1,3),frame(2,3), axes(0,0),axes(1,0),axes(2,0), frame(0,3),frame(1,3),frame(2,3), axes(0,1),axes(1,1),axes(2,1), frame(0,3),frame(1,3),frame(2,3), axes(0,2),axes(1,2),axes(2,2), };
    const GLfloat colors[]  = { 1,0,0, 1,0,0, 0,1,0, 0,1,0, 0,0,1, 0,0,1 }; // Red-Green-Blue

    glLineWidth(2.0f);
    pangolin::glDrawColoredVertices<float,float>(6, vertices, colors, GL_LINES, 3, 3);

    if (frameName.length() > 0) {
        glColor3f(1.0, 1.0, 1.0);
        pangolin::GlText txt = glFont.Text(frameName.c_str());
        txt.Draw(frame(0, 3), frame(1, 3), frame(2, 3));
    }
}

pcl::PointCloud<pcl::Normal> * GlVisualizer::CalculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
{
    // See http://pointclouds.org/documentation/tutorials/normal_estimation.php
    // Initialize a normal estimation object
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    // Pass the cloud to the estimator
    ne.setInputCloud(pc);

    // Set viewpoint from where the normals are calculated
    ne.setViewPoint(0, 0, 0);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal> * cloud_normals = new pcl::PointCloud<pcl::Normal>;

    // Use all neighbors in a sphere of radius 3 cm
    ne.setRadiusSearch(0.3);
    // ne.setKSearch(5); // use the 5 closest points

    // Compute the features
    ne.compute(*cloud_normals);

    return cloud_normals;
}

/*pangolin::GlBuffer * GlVisualizer::UploadPointCloud(pcl::PointCloud<PointTypes::Velodyne>::Ptr &pc)
{
    float pointArray[3*pc->points.size()];
    for (int i = 0; i < pc->points.size(); i++) memcpy((char*)(&pointArray[3*i]), (char*)pc->points[i].data, 3*sizeof(float));

    //pangolin::GlBuffer * glBuf = new pangolin::GlBuffer(pangolin::GlArrayBuffer, pc->points.size(), GL_FLOAT, 3, GL_STATIC_DRAW);
    GLuint bo;

    glGenBuffers(1, &bo);
    glBindBuffer(GL_ARRAY_BUFFER, bo);
    glBufferData(GL_ARRAY_BUFFER, pc->points.size()*3*sizeof(float), 0, GL_STATIC_DRAW);

    //glBuf->Upload(pointArray, 3*sizeof(float)*pc->points.size());
    glBufferSubData(GL_ARRAY_BUFFER, 0, pc->points.size()*3*sizeof(float), pointArray);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return bo;
}*/

void GlVisualizer::DrawPointCloud(pointcloud_t &pointcloud)
//void GlVisualizer::DrawPointCloud(GLuint glBuf, GLuint pointcloud_size)
{
    if (pointcloud.changed) {
        if (pointcloud.pc == 0) {
            pointcloud.changed = false;
            return;
        }

        pointcloud.numberOfPoints = pointcloud.pc->points.size();
        std::vector<color_t> colors;
        for (auto pt : pointcloud.pc->points)
        {
            color_t ptColor;
            ptColor.r = (float)pt.r / 255.0f;
            ptColor.g = (float)pt.g / 255.0f;
            ptColor.b = (float)pt.b / 255.0f;
            colors.push_back(ptColor);
        }

        if (pointcloud.pc_normals)
            if (pointcloud.pc_normals->size() != pointcloud.pc->size())
                pointcloud.pc_normals = 0;

        /*float * ptr = pointcloud.pc->points[0].data;

        float pointArray[3 * pointcloud.pc->points.size()];
        numberOfPoints = 0;
        for (unsigned long i = 0; i < pointcloud.pc->points.size()-1; i++) {
            char * ptr1 = (char *)&pointcloud.pc->points[i];
            char * ptr2 = (char *)&pointcloud.pc->points[i+1];
            if ((ptr2 - ptr1) != sizeof(struct PointTypes::Velodyne)) {
                std::cout << "Pointer suddenly jumped: " << (ptr2 - ptr1) << std::endl;
            }

            if (pointcloud.pc->points[i].ring == pointcloud.ring || pointcloud.ring == -1) {
                //memcpy((char *) (&pointArray[3 * numberOfPoints]), (char *) pointcloud.pc->points[i].data, 3 * sizeof(float));
                memcpy((char *) (&pointArray[3 * numberOfPoints]), (char *)ptr + i*(sizeof(PointTypes::Velodyne)-1), 3 * sizeof(float));
                numberOfPoints++;
            }
        }

        // Upload data to GPU buffer
        if (pointcloud.vbo) glDeleteBuffers(1, &(pointcloud.vbo)); // delete previous buffer
        glGenBuffers(1, &(pointcloud.vbo));
        glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo);
        glBufferData(GL_ARRAY_BUFFER, numberOfPoints * 3 * sizeof(float), 0, GL_STATIC_DRAW);

        //glBuf->Upload(pointArray, 3*sizeof(float)*pc->points.size());
        glBufferSubData(GL_ARRAY_BUFFER, 0, numberOfPoints * 3 * sizeof(float), pointArray);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        */

        // More optimal way of uploading data directly from pointcloud object
        /* Upload data to GPU buffer */
        if (pointcloud.vbo) glDeleteBuffers(1, &(pointcloud.vbo)); // delete previous buffer
        pointcloud.vbo = 0;

        if (pointcloud.vbo_color) glDeleteBuffers(1, &(pointcloud.vbo_color)); // delete previous buffer
        pointcloud.vbo_color = 0;

        if (pointcloud.vbo_normals) glDeleteBuffers(1, &(pointcloud.vbo_normals)); // delete previous buffer
        pointcloud.vbo_normals = 0;

        if (pointcloud.numberOfPoints > 0) {
            glGenBuffers(1, &(pointcloud.vbo)); // generate 1 buffer
            glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo);
            glBufferData(GL_ARRAY_BUFFER, pointcloud.pc->points.size() * sizeof(pcl::PointXYZRGB),
                         pointcloud.pc->points.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);

            /* Colored points : https://github.com/mp3guy/Kintinuous/blob/master/src/utils/PangoCloud.h */
            glGenBuffers(1, &(pointcloud.vbo_color)); // generate 1 buffer
            glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo_color);
            glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(color_t), colors.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);

            if (pointcloud.pc_normals) {
                /* Normal upload */
                glGenBuffers(1, &(pointcloud.vbo_normals)); // generate 1 buffer
                glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo_normals);
                UploadNormals(pointcloud.pc, pointcloud.pc_normals);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            }
        }

        pointcloud.pc = 0;
        pointcloud.changed = false;
    }

    if (pointcloud.numberOfPoints == 0) return;
    if (!pointcloud.vbo) return; // something went wrong, we don't have a VBO to display
    if (!pointcloud.visible) return;
    glPointSize(pointcloud.size);

    /* Render buffered point cloud */
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo);
    glVertexPointer(3, GL_FLOAT, sizeof(pcl::PointXYZRGB), 0); // important to set stride
    glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo_color);
    glColorPointer(3, GL_FLOAT, sizeof(color_t), 0); // important to set stride

    if (pointcloud.dense && shaderProgram)
        glUseProgram(shaderProgram);

    glDrawArrays(GL_POINTS, 0, pointcloud.numberOfPoints);

    if (pointcloud.dense && shaderProgram)
        glUseProgram(0);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    /* Draw normals */
    if (pointcloud.vbo_normals) {
        glEnableClientState(GL_VERTEX_ARRAY);

        glLineWidth(1.0f);
        glBindBuffer(GL_ARRAY_BUFFER, pointcloud.vbo_normals);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glColor4f(0, 0, 1, 1);
        glDrawArrays(GL_LINES, 0, 2*pointcloud.numberOfPoints);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

void GlVisualizer::UploadNormals(const pcl::PointCloud<pcl::PointXYZRGB> * pc, const pcl::PointCloud<pcl::Normal> * pc_normals, float normalScale)
{
    float * pointArray = (float*)malloc(6*pc->points.size() * sizeof(float));
    unsigned long normalCount = 0;
    unsigned long j = 0;
    for (unsigned long i = 0; i < pc->points.size(); i++) {
        Eigen::Vector3f normal = pc_normals->points[i].getNormalVector3fMap();
        //float normal_length = normal.norm();
        Eigen::Vector3f normalized_normal = normal.normalized();

        if (!normal.hasNaN()) {
            //std::cout << "Normal " << i << std::endl << normalized_normal << std::endl << std::endl;

            //memcpy((char*)(&pointArray[6*i]), (char*)pc->points[i].data, 3*sizeof(float));
            pointArray[j + 0] = pc->points[i].x + 0.01*normalScale*normalized_normal[0];
            pointArray[j + 1] = pc->points[i].y + 0.01*normalScale*normalized_normal[1];
            pointArray[j + 2] = pc->points[i].z + 0.01*normalScale*normalized_normal[2];

            pointArray[j + 3] = pointArray[j + 0] + normalScale * normalized_normal[0];
            pointArray[j + 4] = pointArray[j + 1] + normalScale * normalized_normal[1];
            pointArray[j + 5] = pointArray[j + 2] + normalScale * normalized_normal[2];

            normalCount++;
            j += 6;
        }
    }

    /*pangolin::GlBuffer * glBuf = new pangolin::GlBuffer(pangolin::GlArrayBuffer, 2*normalCount, GL_FLOAT, 3, GL_STATIC_DRAW);
    glBuf->Upload(pointArray, 6*sizeof(float)*normalCount);*/
    glBufferData(GL_ARRAY_BUFFER, 6 * normalCount * sizeof(float), pointArray, GL_STATIC_DRAW);
}

void GlVisualizer::DrawNormals(pangolin::GlBuffer * glBuf)
{
    glLineWidth(1.0f);
    glBuf->Bind();
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);

    glColor4f(0,0,1,1);
    glDrawArrays(GL_LINES, 0, glBuf->num_elements);

    glDisableClientState(GL_VERTEX_ARRAY);
    glBuf->Unbind();
}

void GlVisualizer::RegisterExitHandler(const boost::function<void (int s)> &handler)
{
    ExitHandler_ = handler;
}

void GlVisualizer::Exit(void)
{
    shouldExit = true;
    if (VisualizerThread->joinable())
        VisualizerThread->join();
}

bool GlVisualizer::isRunning(void)
{
    return isRunning_ || !guiIsReady_;
}

bool GlVisualizer::isThreadRunning(void)
{
    return VisualizerThread->try_join_for(boost::chrono::nanoseconds(1));
}

void GlVisualizer::AddFrame(Eigen::Transform<float,3,Eigen::Affine> tfFrame, const std::string &frameName, float axesLength)
{
    frame_t frame;
    frame.name = frameName;
    frame.transform = tfFrame.matrix();
    frame.axes_length = axesLength;
    Frames.push_back(frame);
}

void GlVisualizer::UpdateFrame(const std::string &frameName, Eigen::Transform<double,3,Eigen::Affine> tfFrame)
{
    std::vector<frame_t>::const_iterator it = std::find_if(Frames.begin(), Frames.end(), find_frame(frameName));
    if (it != Frames.end()) {
        frame_t *frame = (frame_t *) &*it;
        frame->transform = tfFrame.matrix().cast<float>();
    }
}

void GlVisualizer::UpdateFrame(const std::string &frameName, Eigen::Transform<float,3,Eigen::Affine> tfFrame)
{
    std::vector<frame_t>::const_iterator it = std::find_if(Frames.begin(), Frames.end(), find_frame(frameName));
    if (it != Frames.end()) {
        frame_t *frame = (frame_t *) &*it;
        frame->transform = tfFrame.matrix();
    }
}

void GlVisualizer::ClearFrames(void)
{
    Frames.clear();
}

void GlVisualizer::AddTrajectory(const std::string &trajectoryName)
{
    trajectory_t trajectory;
    trajectory.name = trajectoryName;
    Trajectories.push_back(trajectory);
}

void GlVisualizer::UpdateTrajectory(const std::string &trajectoryName, const std::vector<Eigen::Matrix4f> &poses)
{
    std::vector<trajectory_t>::const_iterator it = std::find_if(Trajectories.begin(), Trajectories.end(), find_trajectory(trajectoryName));
    if (it != Trajectories.end()) {
        trajectory_t *trajectory = (trajectory_t *) &*it;
        trajectory->poses = poses;
    }
}

void GlVisualizer::ClearTrajectories(void)
{
    Trajectories.clear();
}



// https://learnopengl.com/Getting-started/Shaders
int GlVisualizer::ConfigureShaders(const char* vShaderCode, const char* fShaderCode)
{
    unsigned int vertex, fragment;
    int success;
    char infoLog[512];
    unsigned int ID;

    // vertex Shader
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderCode, NULL);
    glCompileShader(vertex);
    // print compile errors if any
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(vertex, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    // vertex Shader
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderCode, NULL);
    glCompileShader(fragment);
    // print compile errors if any
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        glGetShaderInfoLog(vertex, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    // shader Program
    ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    glLinkProgram(ID);
    // print linking errors if any
    glGetProgramiv(ID, GL_LINK_STATUS, &success);
    if(!success)
    {
        glGetProgramInfoLog(ID, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    // delete the shaders as they're linked into our program now and no longer necessery
    glDeleteShader(vertex);
    glDeleteShader(fragment);

    return ID; // return the program ID
}

void GlVisualizer::setBool(unsigned int ProgramID, const std::string &name, bool value) const
{
    glUniform1i(glGetUniformLocation(ProgramID, name.c_str()), (int)value);
}
void GlVisualizer::setInt(unsigned int ProgramID, const std::string &name, int value) const
{
    glUniform1i(glGetUniformLocation(ProgramID, name.c_str()), value);
}
void GlVisualizer::setFloat(unsigned int ProgramID, const std::string &name, float value) const
{
    glUniform1f(glGetUniformLocation(ProgramID, name.c_str()), value);
}

void GlVisualizer::AddSliderDouble(const std::string &sliderName, double min, double max, double init, bool locked)
{
    if (!toolboxEnabled_) return;
    boost::unique_lock<boost::mutex> lock(slider_mtx);

    slider_t slider;
    slider.name = boost::algorithm::to_lower_copy(sliderName);
    slider.min = min;
    slider.max = max;
    slider.valueDouble = init;
    slider.valueInt = -1;
    slider.Double = true;
    if (locked)
        slider.pangolinVarDouble = new pangolin::Var<double>("ui." + slider.name, init);
    else
        slider.pangolinVarDouble = new pangolin::Var<double>("ui." + slider.name, init, min, max, false);
    slider.pangolinVarInt = 0;
    Sliders.push_back(slider);
}

void GlVisualizer::AddSliderInt(const std::string &sliderName, int min, int max, int init, bool locked)
{
    if (!toolboxEnabled_) return;
    boost::unique_lock<boost::mutex> lock(slider_mtx);

    slider_t slider;
    slider.name = boost::algorithm::to_lower_copy(sliderName);
    slider.min = min;
    slider.max = max;
    slider.locked = locked;
    slider.valueDouble = -1;
    slider.valueInt = init;
    slider.Double = false;
    slider.pangolinVarDouble = 0;
    if (locked)
        slider.pangolinVarInt = new pangolin::Var<int>("ui." + slider.name, init);
    else
        slider.pangolinVarInt = new pangolin::Var<int>("ui." + slider.name, init, min, max, false);
    Sliders.push_back(slider);
}

void GlVisualizer::ChangeSliderBounds(const std::string &sliderName, double min, double max)
{
    if (!toolboxEnabled_) return;
    boost::unique_lock<boost::mutex> lock(slider_mtx);

    GlVisualizer::slider_t * slider = FindSlider(boost::algorithm::to_lower_copy(sliderName));
    if (slider) {
        if (slider->Double) {
            slider->pangolinVarDouble->Meta().range[0] = min;
            slider->pangolinVarDouble->Meta().range[1] = max;
            // Correct current value - saturate and update if necessary
            if (*slider->pangolinVarDouble < min) *slider->pangolinVarDouble = min;
            if (*slider->pangolinVarDouble > max) *slider->pangolinVarDouble = max;
            slider->valueDouble = *slider->pangolinVarDouble;
        } else {
            slider->pangolinVarInt->Meta().range[0] = (int)min;
            slider->pangolinVarInt->Meta().range[1] = (int)max;
            // Correct current value - saturate and update if necessary
            if (*slider->pangolinVarInt < min) *slider->pangolinVarInt = min;
            if (*slider->pangolinVarInt > max) *slider->pangolinVarInt = max;
            slider->valueInt = *slider->pangolinVarInt;
        }
    }
}

void GlVisualizer::SetSlider(const std::string &sliderName, double value)
{
    if (!toolboxEnabled_) return;
    boost::unique_lock<boost::mutex> lock(slider_mtx);

    GlVisualizer::slider_t * slider = FindSlider(boost::algorithm::to_lower_copy(sliderName));
    if (slider) {
        if (slider->Double) {
            //slider->pangolinVarDouble->Ref().Set(value);
            //slider->pangolinVarDouble->Meta().gui_changed = true;
            *slider->pangolinVarDouble = value;
            slider->valueDouble = value;
        } else {
            //slider->pangolinVarInt->Ref().Set((int)value);
            //slider->pangolinVarInt->Meta().gui_changed = true;
            *slider->pangolinVarInt = (int)value;
            slider->valueInt = (int)value;
        }
    }
}

GlVisualizer::slider_t * GlVisualizer::FindSlider(const std::string slider_name)
{
    std::vector<GlVisualizer::slider_t>::const_iterator it = std::find_if(Sliders.begin(), Sliders.end(), find_slider(slider_name));
    if (it != Sliders.end())
        return (GlVisualizer::slider_t *)&*it;
    else
        return 0;
}

double GlVisualizer::GetSlider(const std::string &sliderName)
{
    if (!toolboxEnabled_) return -1;

    boost::unique_lock<boost::mutex> lock(slider_mtx);
    GlVisualizer::slider_t * slider = FindSlider(boost::algorithm::to_lower_copy(sliderName));
    if (slider) {
        if (slider->Double)
            return slider->valueDouble;
        else
            return (double) std::round(slider->valueInt);
    } else
        return -1;
}

void GlVisualizer::AddCheckbox(const std::string &checkboxName, bool init)
{
    if (!toolboxEnabled_) return;
    boost::unique_lock<boost::mutex> lock(checkbox_mtx);

    checkbox_t checkbox;
    checkbox.name = boost::algorithm::to_lower_copy(checkboxName);
    checkbox.value = init;
    checkbox.pangolinVar = new pangolin::Var<bool>("ui." + checkbox.name, init, true);
    Checkboxes.push_back(checkbox);
}

GlVisualizer::checkbox_t * GlVisualizer::FindCheckbox(const std::string checkbox_name)
{
    std::vector<GlVisualizer::checkbox_t>::const_iterator it = std::find_if(Checkboxes.begin(), Checkboxes.end(), find_checkbox(checkbox_name));
    if (it != Checkboxes.end())
        return (GlVisualizer::checkbox_t *)&*it;
    else
        return 0;
}

bool GlVisualizer::GetCheckbox(const std::string &checkboxName)
{
    if (!toolboxEnabled_) return -1;

    boost::unique_lock<boost::mutex> lock(checkbox_mtx);
    GlVisualizer::checkbox_t * checkbox = FindCheckbox(boost::algorithm::to_lower_copy(checkboxName));
    if (checkbox) {
        return checkbox->value;
    } else
        return false;
}

void GlVisualizer::AddLabel(const std::string &labelName, const std::string &labelString)
{
    if (!toolboxEnabled_) return;
    boost::unique_lock<boost::mutex> lock(label_mtx);

    label_t label;
    label.name = boost::algorithm::to_lower_copy(labelName);
    label.pangolin = new pangolin::Var<std::string>("ui." + label.name, labelString, pangolin::META_FLAG_READONLY);

    Labels.push_back(label);
}

void GlVisualizer::SetLabel(const std::string &labelName, const std::string &labelString)
{
    if (!toolboxEnabled_) return;
    boost::unique_lock<boost::mutex> lock(label_mtx);

    GlVisualizer::label_t * label = FindLabel(boost::algorithm::to_lower_copy(labelName));
    if (label) {
        *label->pangolin = labelString;
    }
}

GlVisualizer::label_t * GlVisualizer::FindLabel(const std::string label_name)
{
    std::vector<GlVisualizer::label_t>::const_iterator it = std::find_if(Labels.begin(), Labels.end(), find_label(label_name));
    if (it != Labels.end())
        return (GlVisualizer::label_t *)&*it;
    else
        return 0;
}

void GlVisualizer::AddButton(const std::string &buttonName)
{
    if (!toolboxEnabled_) return;
    boost::unique_lock<boost::mutex> lock(button_mtx);

    button_t button;
    button.name = boost::algorithm::to_lower_copy(buttonName);
    button.pangolin = new pangolin::Var<bool>("ui." + button.name, false, false);

    Buttons.push_back(button);
}

void GlVisualizer::AddButton(const std::string &buttonName, const boost::function<void ()> &handler)
{
    AddButton(buttonName);
    RegisterButtonHandler(buttonName, handler);
}

GlVisualizer::button_t * GlVisualizer::FindButton(const std::string buttonName)
{
    std::vector<GlVisualizer::button_t>::const_iterator it = std::find_if(Buttons.begin(), Buttons.end(), find_button(buttonName));
    if (it != Buttons.end())
        return (GlVisualizer::button_t *)&*it;
    else
        return 0;
}

void GlVisualizer::RegisterButtonHandler(const std::string &buttonName, const boost::function<void ()> &handler)
{
    boost::unique_lock<boost::mutex> lock(button_mtx);
    GlVisualizer::button_t * button = FindButton(boost::algorithm::to_lower_copy(buttonName));
    if (button) {
        button->handler = handler;
    }
}

void GlVisualizer::RegisterMouseButtonHandler(GlVisualizer::mouseButton_t button, const boost::function<void (Eigen::Vector3d)>& func, GlVisualizer::keyboardButton_t keyCondition)
{
    if (handler) {
        handler->RegisterMouseButtonHandler(static_cast<pangolin::GlHandler3D::mouseButton_t>(button), func, static_cast<pangolin::GlHandler3D::keyboardButton_t>(keyCondition));
    }
}

Eigen::Vector3d GlVisualizer::GetMouse3Dposition()
{
    return handler->getMouse3Dposition();
}

void GlVisualizer::RegisterSingleHighlightHandler(const boost::function<void (Eigen::Vector3d, bool, bool)>& func)
{
    if (handler) {
        handler->RegisterHighlightSingle(func);
    }
}

void GlVisualizer::RegisterDrawingHighlightHandler(const boost::function<void (Eigen::Vector3d, bool, bool)>& func)
{
    if (handler) {
        handler->RegisterHighlightDrawing(func);
    }
}

void GlVisualizer::RegisterKeyboardButtonHandler(GlVisualizer::keyboardButtonGroup_t group, const boost::function<void (char)>& func, GlVisualizer::keyboardButton_t keyCondition)
{
    if (handler) {
        handler->RegisterKeyboardButtonHandler(static_cast<pangolin::GlHandler3D::keyboardButtonGroup_t>(group), func, static_cast<pangolin::GlHandler3D::keyboardButton_t>(keyCondition));
    }
}
