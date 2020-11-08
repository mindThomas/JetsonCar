#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "imutab.h"
#include "realsensetab.h"
#include "zedtab.h"
#include "motortab.h"
#include "testtab.h"
#include "loggertab.h"
#include "window.h"

#include <QSettings>
#include <QDesktopWidget>

#include <Qt3DCore/QEntity>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>
#include <Qt3DCore/QTransform>
#include <Qt3DCore/QAspectEngine>

#include <Qt3DInput/QInputAspect>

#include <Qt3DRender/QRenderAspect>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QTorusMesh>

#include "qt3dwindow.h"
#include "qorbitcameracontroller.h"

#include "qglViewerTest.h"

#include "QGLVisualizer.h"
#include "imu/imu.h"
#include "realsense/realsense.h"
#include "zed/zed.h"

class Vesc3DView;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    Vesc3DView *m3dView;
    QGLVisualizer * qglVisualizer;

    IMU imu;
    Realsense realsense;
    ZED zed;
};
#endif // MAINWINDOW_H
