#ifndef REALSENSETAB_H
#define REALSENSETAB_H

#include <QWidget>
#include <QSettings>
#include <QDesktopWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QComboBox>

#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <math.h>
#include <float.h>
#include <iostream>

#include "cvimage.h"

#include "realsense/realsense.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#define deg2rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2deg(angleRadians) ((angleRadians) * 180.0 / M_PI)

namespace Ui {
class RealsenseTab;
}

class RealsenseTab : public QWidget
{
    Q_OBJECT

public:
    explicit RealsenseTab(Realsense * realsense_, QWidget *parent = nullptr);
    ~RealsenseTab();

private slots:
    void on_getOrientationBtn_clicked();
    void on_getViewBtn_clicked();
    void on_resetOriginBtn_clicked();
    void on_resetViewBtn_clicked();

private:
    void ResetView();
    void PoseUpdate(Eigen::Affine3f pose);
    void ImageUpdate(uint8_t side, cv::Mat image);
    void SaveImage(bool side);

    Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 3, 3> rotm);
    Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 4, 4> transform);

private:
    Ui::RealsenseTab *ui;

    QComboBox * rectificationSetting;

    cvImage leftImage;
    cvImage rightImage;
    Realsense * realsense;

    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::xfeatures2d::SIFT> sift;
};

#endif // REALSENSETAB_H
