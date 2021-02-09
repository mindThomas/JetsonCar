#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QMessageBox>
#include <QSettings>
#include <QDesktopWidget>
#include <QFileDialog>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <jetsoncar_utils/jetsoncar_utils.h>
#include <jetsoncar_utils/formatting.hpp>
#include <spdlog/stopwatch.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

const auto console = spdlog::stdout_color_st("MainWindow");

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    spdlog::set_level(spdlog::level::info); // Set global log level to info
    setWindowTitle(QStringLiteral("Pointcloud Visualizer"));    

    // Add origin frame to pointcloud view
    ui->qglVisualizer->AddFrame(Eigen::Affine3f::Identity(), "Origin");
    ui->qglVisualizer->SetViewpoint(Eigen::Vector3f(0, 0, 90), Eigen::Vector3f(0, 0, -6));

    ui->qglVisualizer->AddPointcloud("Loaded");
    ui->qglVisualizer->AddPointcloud("Test", 10);

    // Add test pointcloud after 2 seconds
    spdlog::stopwatch sw;
    console->info("Elapsed: {}", sw);
    QTimer::singleShot(2000, this, [&]() {
        console->info("Elapsed: {}", sw);
        checkForReady();
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::checkForReady(void)
{
    auto point = [](float x, float y, float z, uint8_t R, uint8_t G, uint8_t B) {
        pcl::PointXYZRGB pt(R, G, B);
        pt.x = x;
        pt.y = y;
        pt.z = z;
        return pt;
    };

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl->push_back(point(0.0, 0.0, 0.0, 255, 255, 255));
    pcl->push_back(point(1.0, 0.0, 0.0, 255, 0, 0));
    pcl->push_back(point(0.0, 1.0, 0.0, 0, 255, 0));
    pcl->push_back(point(0.0, 0.0, 1.0, 0, 0, 255));

    spdlog::info("Added test pointcloud");

    ui->qglVisualizer->UpdatePointcloud("Test", pcl);
}

void MainWindow::on_actionOpen_PCD_triggered()
{
    QString FilePath = QFileDialog::getOpenFileName(this, "Select a pointcloud", ".", "Pointcloud (*.pcd)");

    if (FilePath.isEmpty()) {
        QMessageBox::critical( this, "Open PCD", QString("File not specified!"), QMessageBox::Ok );
        return;
    }

    if (!utils::FileExist(FilePath.toStdString())) {
        QMessageBox::critical( this, "Open PCD", QString("File does not exist {%1}!").arg(FilePath), QMessageBox::Ok );
        return;
    }

    if (utils::getFileExtension(FilePath.toStdString()).compare(".pcd")) {
        QMessageBox::critical( this, "Open PCD", QString("Incorrect file type %1 (should be .pcd)").arg(QString::fromStdString(utils::getFileExtension(FilePath.toStdString()))), QMessageBox::Ok );
        return;
    }

    auto pc_new = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto ret = pcl::io::loadPCDFile(FilePath.toStdString(), *pc_new);
    REQUIRE(ret == 0, "Could not parse PCD file (unknown format).");

    if (ret != 0) {
        QMessageBox::critical( this, "Open PCD", QString("Could not parse PCD file (unknown format)."), QMessageBox::Ok );
        return;
    }

    spdlog::info("Opened pointcloud: {}", FilePath.toStdString());

    ui->qglVisualizer->UpdatePointcloud("Loaded", pc_new);
    ui->qglVisualizer->SetPointcloudVisibility("Test", false);
    ui->qglVisualizer->AdjustToScene(ui->qglVisualizer->EstimateLongestAxisFromOrientedBoundingBox(*pc_new));
}
