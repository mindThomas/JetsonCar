#include "QTPCLVisualizer.h"
#include "ui_QTPCLVisualizer.h"

QTPCLVisualizer::QTPCLVisualizer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::QTPCLVisualizer)
{
    ui->setupUi(this);

    this->setWindowTitle ("PCL viewer");

    // Setup the cloud pointer
    cloud.reset (new PointCloudT);
    // The number of points in the cloud
    cloud->points.resize (200);

    // The default color
    red   = 128;
    green = 128;
    blue  = 128;

    // Fill the cloud with some points
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

        cloud->points[i].r = red;
        cloud->points[i].g = green;
        cloud->points[i].b = blue;
    }

    // Set up the QVTK window
    /*viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    viewer->addPointCloud (cloud, "cloud");
    viewer->resetCamera ();
    ui->qvtkWidget->update ();*/
}

QTPCLVisualizer::~QTPCLVisualizer()
{
    delete ui;
}

void QTPCLVisualizer::SetViewer(pcl::visualization::PCLVisualizer * viewer)
{
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
}