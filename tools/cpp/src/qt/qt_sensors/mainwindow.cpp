#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    imu.Connect();
    realsense.Connect();
    zed.Connect();

    ui->tabWidget->addTab(new ImuTab(&imu, &realsense), "IMU");
    ui->tabWidget->addTab(new RealsenseTab(&realsense), "Realsense");
    ui->tabWidget->addTab(new ZedTab(&zed), "ZED");
    ui->tabWidget->addTab(new MotorTab, "Motor");
    ui->tabWidget->addTab(new LoggerTab(&imu, &realsense, &zed), "Logger");
}

MainWindow::~MainWindow()
{
    delete ui;
}
