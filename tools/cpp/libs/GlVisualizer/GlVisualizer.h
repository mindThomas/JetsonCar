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

#ifndef GLVISUALIZER_
#define GLVISUALIZER_

#include <stdio.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pangolin/pangolin.h>
#include "GlHandler3D.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define MAIN_AXIS_LENGTH 1.0f
#define FRAME_AXIS_LENGTH 0.2f
#define TRAJECTORY_AXIS_LENGTH 0.1f
#define AXIS_FONT_SIZE	16.0f
#define POINT_SIZE_NORMAL   1
#define POINT_SIZE_HIGHLIGHTED  3

class GlVisualizer {
    public:
        GlVisualizer();

        GlVisualizer(bool slidersEnabled);

        GlVisualizer(const std::string &screenName);
        GlVisualizer(char * screenName);

        GlVisualizer(const std::string &screenName, bool slidersEnabled);

        void Display(void);

        void OpenPCD(const std::string &pcd_file);

        void AddPointcloud(const std::string pointcloud_name, uint8_t size = POINT_SIZE_NORMAL);
        void AddPointcloud(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc, uint8_t size = POINT_SIZE_NORMAL);
        void AddDensePointcloud(const std::string pointcloud_name);

        void UpdatePointcloud(const std::string pointcloud_name, pcl::PointCloud <pcl::PointXYZRGB> *new_pc);
        void UpdatePointcloudWithNormals(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB> *new_pc, pcl::PointCloud<pcl::Normal> *new_pc_normals);

        void UpdatePointcloud(const std::string pointcloud_name,
                              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc);
        void UpdatePointcloudWithNormals(const std::string pointcloud_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc, const pcl::PointCloud<pcl::Normal>::Ptr &new_pc_normals);

        void ChangePointSize(const std::string pointcloud_name, uint8_t point_size);

        void RemovePointcloud(const std::string pointcloud_name);

        void ClearPointclouds();

        void
        UpdatePointcloudWithAxes(const std::string pointcloud_name, pcl::PointCloud <pcl::PointXYZRGB> *new_pc);

        void SetPointcloudVisibility(const std::string pointcloud_name, bool visible);

        void VisualizeNormals(void);

        void RegisterExitHandler(const boost::function<void (int s)> &handler);

        void SetViewpoint(Eigen::Matrix<float, 4, 4> view);
        void SetDefaultViewpoint();
        Eigen::Matrix<float, 4, 4> GetViewpoint(void);

        void AddFrame(Eigen::Transform<float, 3, Eigen::Affine> tfFrame, const std::string &frameName, float axesLength = FRAME_AXIS_LENGTH);

        void UpdateFrame(const std::string &frameName, Eigen::Transform<double, 3, Eigen::Affine> tfFrame);
        void UpdateFrame(const std::string &frameName, Eigen::Transform<float, 3, Eigen::Affine> tfFrame);

        void ClearFrames(void);

        void AddTrajectory(const std::string &trajectoryName);

        void UpdateTrajectory(const std::string &trajectoryName, const std::vector<Eigen::Matrix4f> &poses);

        void ClearTrajectories(void);

        void AddSliderDouble(const std::string &sliderName, double min, double max, double init, bool locked = false);

        void AddSliderInt(const std::string &sliderName, int min, int max, int init, bool locked = false);

        void ChangeSliderBounds(const std::string &sliderName, double min, double max);

        void SetSlider(const std::string &sliderName, double value);

        double GetSlider(const std::string &sliderName);

        void AddButton(const std::string &buttonName);
        void AddButton(const std::string &buttonName, const boost::function<void ()> &handler);

        void AddLabel(const std::string &labelName, const std::string &labelString = "");
        void SetLabel(const std::string &labelName, const std::string &labelString);

        void AddCheckbox(const std::string &checkboxName, bool init);
        bool GetCheckbox(const std::string &checkboxName);

        void RegisterButtonHandler(const std::string &buttonName, const boost::function<void ()> &handler);

        void Exit(void);

        bool isRunning(void);

        bool isThreadRunning(void);

        int GetKeyState(void);


        /* Button handlers */
        typedef enum mouseButton_t : int {
            Left = pangolin::MouseButtonLeft,
            Middle = pangolin::MouseButtonMiddle,
            Right = pangolin::MouseButtonRight,
            WheelUp = pangolin::MouseWheelUp,
            WheelDown = pangolin::MouseWheelDown,
            WheelRight = pangolin::MouseWheelRight,
            WheelLeft = pangolin::MouseWheelLeft
        } mouseButton_t;

        typedef enum keyboardButton_t : int {
            None = 0,
            Shift = pangolin::KeyModifierShift,
            Ctrl = pangolin::KeyModifierCtrl,
            Alt = pangolin::KeyModifierAlt,
            Cmd = pangolin::KeyModifierCmd,
            Fnc = pangolin::KeyModifierFnc
        } keyboardButton_t;

        typedef enum keyboardButtonGroup_t : int {
            Characters = 0,
            Numbers
        } keyboardButtonGroup_t;

        void RegisterMouseButtonHandler(mouseButton_t button, const boost::function<void (Eigen::Vector3d)>& func, keyboardButton_t keyCondition = None);
        Eigen::Vector3d GetMouse3Dposition();
        void RegisterSingleHighlightHandler(const boost::function<void (Eigen::Vector3d, bool, bool)>& func);
        void RegisterDrawingHighlightHandler(const boost::function<void (Eigen::Vector3d, bool, bool)>& func);
        void RegisterKeyboardButtonHandler(keyboardButtonGroup_t group, const boost::function<void (char)>& func, keyboardButton_t keyCondition = None);

        typedef enum DockLocation_t {
            TopLeft = 0,
            TopCenter,
            TopRight,
            MidLeft,
            MidCenter,
            MidRight,
            BottomLeft,
            BottomCenter,
            BottomRight,
            Top,
            Mid,
            Bottom
        } DockLocation_t;

        void AddImage(const std::string image_name, unsigned int width, unsigned int height, DockLocation_t dock = TopRight, double widthPct = 0);
        void UpdateImage(const std::string image_name, cv::Mat &img);

    private:
        typedef struct EIGEN_ALIGN16 color_t {
            float r;
            float g;
            float b;
            float dummy;
        } color_t;
        typedef struct pointcloud_t {
            std::string name;
            pcl::PointCloud <pcl::PointXYZRGB> *pc{0};
            pcl::PointCloud <pcl::Normal> *pc_normals{0};
            GLuint vbo;
            GLuint vbo_color;
            GLuint vbo_normals;
            bool visible;
            bool dense; // will use shader for height coloring
            bool changed;
            uint8_t size;
            unsigned long numberOfPoints;
            Eigen::Matrix<float, 4, 1> centroid;
            Eigen::Matrix<float, 3, 3> covariance;
            Eigen::Matrix<float, 3, 3> eigenvectors;
            Eigen::Matrix<float, 3, 1> eigenvalues;
        } pointcloud_t;

        typedef struct checkbox_t {
            std::string name;
            bool value;
            pangolin::Var<bool> *pangolinVar;
        } checkbox_t;

        typedef struct slider_t {
            std::string name;
            double min;
            double max;
            double valueDouble;
            int valueInt;
            bool Double;
            bool locked;
            pangolin::Var<int> *pangolinVarInt;
            pangolin::Var<double> *pangolinVarDouble;
        } slider_t;

        typedef struct button_t {
            std::string name;
            boost::function<void ()> handler;
            pangolin::Var<bool> *pangolin;
        } button_t;

        typedef struct label_t {
            std::string name;
            pangolin::Var<std::string> *pangolin;
        } label_t;

        typedef struct trajectory_t {
            std::string name;
            std::vector<Eigen::Matrix4f> poses;
        } trajectory_t;

        typedef struct frame_t {
            std::string name;
            Eigen::Matrix4f transform;
            float axes_length;
        } frame_t;

        typedef struct image_t {
            std::string name;
            unsigned int width;
            unsigned int height;
            DockLocation_t dock;
            double widthPct;
            cv::Mat image;
            pangolin::View *view;
            pangolin::GlTexture *texture;
            unsigned char *data;
        } image_t;

        struct find_pointcloud : std::unary_function<pointcloud_t, bool> {
            std::string name;

            find_pointcloud(std::string name) : name(name) {}

            bool operator()(const pointcloud_t &s) const {
                return s.name == name;
            }
        };

        struct find_frame : std::unary_function<frame_t, bool> {
            std::string name;

            find_frame(std::string name) : name(name) {}

            bool operator()(const frame_t &s) const {
                return s.name == name;
            }
        };

        struct find_trajectory : std::unary_function<trajectory_t, bool> {
            std::string name;

            find_trajectory(std::string name) : name(name) {}

            bool operator()(const trajectory_t &s) const {
                return s.name == name;
            }
        };

        struct find_slider : std::unary_function<slider_t, bool> {
            std::string name;

            find_slider(std::string name) : name(name) {}

            bool operator()(const slider_t &s) const {
                return s.name == name;
            }
        };

        struct find_checkbox : std::unary_function<checkbox_t, bool> {
            std::string name;

            find_checkbox(std::string name) : name(name) {}

            bool operator()(const checkbox_t &s) const {
                return s.name == name;
            }
        };

        struct find_label : std::unary_function<label_t, bool> {
            std::string name;

            find_label(std::string name) : name(name) {}

            bool operator()(const label_t &s) const {
                return s.name == name;
            }
        };        

        struct find_button : std::unary_function<button_t, bool> {
            std::string name;

            find_button(std::string name) : name(name) {}

            bool operator()(const button_t &s) const {
                return s.name == name;
            }
        };

        struct find_image : std::unary_function<image_t, bool> {
            std::string name;

            find_image(std::string name) : name(name) {}

            bool operator()(const image_t &s) const {
                return s.name == name;
            }
        };

        std::string screenName_;
        std::unique_ptr<boost::thread> VisualizerThread; // using unique_ptr helps deleting the object when the thread terminates
        std::unique_ptr<pangolin::GlHandler3D> handler;
        bool guiIsReady_;
        std::unique_ptr<pangolin::OpenGlRenderState> s_cam;
        bool toolboxEnabled_;
        unsigned int sidebarWidth_;
        std::vector<slider_t> Sliders;
        boost::mutex slider_mtx;
        std::vector<button_t> Buttons;
        boost::mutex button_mtx;
        std::vector<label_t> Labels;
        boost::mutex label_mtx;        
        std::vector<checkbox_t> Checkboxes;
        boost::mutex checkbox_mtx;
        std::vector<pointcloud_t> Pointclouds;
        std::vector<GLuint> VBOForClearance;
        boost::mutex frame_mtx;
        pangolin::GlFont glFont;
        unsigned int shaderProgram;
        int keyState;

        Eigen::Transform<float, 3, Eigen::Affine> tfLiDAR2Body;
        pangolin::OpenGlMatrix Viewport;
        Eigen::Matrix<float, 4, 4> defaultViewpoint;

        std::vector<frame_t> Frames;
        std::vector<trajectory_t> Trajectories;
        std::vector<image_t> Images;

        bool shouldExit;
        bool isRunning_;

        boost::function<void (int s)> ExitHandler_;

        void Visualize(void);

        void AddPointcloud(const std::string pointcloud_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &new_pc,
                           float R, float G, float B, uint8_t size, int ring);

        pointcloud_t *FindPointcloud(const std::string pointcloud_name);

        void DrawFrame(Eigen::Matrix4f frame, const std::string &frameName, float axis_length);

        void
        DrawFrame(Eigen::Matrix4f frame, const std::string &frameName, float axis_length_x, float axis_length_y,
                  float axis_length_z);

        void DrawPointCloud(pointcloud_t &pc);

        pcl::PointCloud <pcl::Normal> *CalculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc);

        void UploadNormals(const pcl::PointCloud<pcl::PointXYZRGB> * pc, const pcl::PointCloud<pcl::Normal> * pc_normals, float normalScale = 1.0f);

        void DrawNormals(pangolin::GlBuffer *glBuf);

        slider_t *FindSlider(const std::string slider_name);
        button_t * FindButton(const std::string buttonName);
        label_t * FindLabel(const std::string label_name);
        checkbox_t * FindCheckbox(const std::string checkbox_name);
        image_t * FindImage(const std::string image_name);

        int ConfigureShaders(const char *vShaderCode, const char *fShaderCode);

        void setBool(unsigned int ProgramID, const std::string &name, bool value) const;

        void setInt(unsigned int ProgramID, const std::string &name, int value) const;

        void setFloat(unsigned int ProgramID, const std::string &name, float value) const;
};

#endif
