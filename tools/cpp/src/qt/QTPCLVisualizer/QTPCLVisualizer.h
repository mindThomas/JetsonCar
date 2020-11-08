#ifndef QTPCLVISUALIZER_H
#define QTPCLVISUALIZER_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class QTPCLVisualizer;
}

class QTPCLVisualizer : public QMainWindow
{
    Q_OBJECT

public:
    explicit QTPCLVisualizer(QWidget *parent = nullptr);
    ~QTPCLVisualizer();

    void SetViewer(pcl::visualization::PCLVisualizer * viewer);

private:
    Ui::QTPCLVisualizer *ui;

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    PointCloudT::Ptr cloud;

    unsigned int red;
    unsigned int green;
    unsigned int blue;
};

#endif // QTPCLVISUALIZER_H
