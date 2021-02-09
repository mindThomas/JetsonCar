#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "imutab.h"
#include "testtab.h"
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
#include <jetsoncar_utils/jetsoncar_utils.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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

private slots:
    void on_actionOpen_PCD_file_triggered();

private:
    Qt3DCore::QEntity* createTestScene();

    pcl::PointXYZRGB point(float x, float y, float z, uint8_t R, uint8_t G, uint8_t B);
    void checkForReady(void);    

private:
    Ui::MainWindow *ui;

    Vesc3DView *m3dView;
    QGLVisualizer * qglVisualizer;
};
#endif // MAINWINDOW_H
