#ifndef ZEDTAB_H
#define ZEDTAB_H

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

#include "zed/zed.h"

#define deg2rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2deg(angleRadians) ((angleRadians) * 180.0 / M_PI)

namespace Ui {
class ZedTab;
}

class ZedTab : public QWidget
{
    Q_OBJECT

public:
    explicit ZedTab(ZED * zed_, QWidget *parent = nullptr);
    ~ZedTab();

private slots:
    void on_getOrientationBtn_clicked();

private slots:
    void on_getViewBtn_clicked();
    void on_resetOriginBtn_clicked();

    void on_resetViewBtn_clicked();

private:
    void ResetView();
    void PoseUpdate(ZED::Pose pose);
    void ImageUpdate(uint8_t side, cv::Mat image);
    void SaveImage(bool side);

    Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 3, 3> rotm);
    Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 4, 4> transform);

private:
    Ui::ZedTab *ui;

    QComboBox * rectificationSetting;

    cvImage leftImage;
    cvImage rightImage;
    ZED * zed;
};

#endif // ZEDTAB_H
