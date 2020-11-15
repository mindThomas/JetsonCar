#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    imu.Connect();
    realsense.Connect();

    #ifdef WITH_ZED_CAMERA
    zed.Connect();
    #endif

    ui->tabWidget->addTab(new ImuTab(&imu, &realsense), "IMU");
    ui->tabWidget->addTab(new RealsenseTab(&realsense), "Realsense");

    #ifdef WITH_ZED_CAMERA
    ui->tabWidget->addTab(new ZedTab(&zed), "ZED");
    #endif
    
    ui->tabWidget->addTab(new MotorTab, "Motor");

    #ifdef WITH_ZED_CAMERA
    ui->tabWidget->addTab(new LoggerTab(&imu, &realsense, &zed), "Logger");
    #else
    ui->tabWidget->addTab(new LoggerTab(&imu, &realsense), "Logger");
    #endif
}

MainWindow::~MainWindow()
{
    delete ui;
}
