#ifndef CVIMAGE_H
#define CVIMAGE_H

#include <QWidget>
#include <QLabel>
#include <QGridLayout>
#include <QTimer>
#include <QPixmap>
#include <QDebug>
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace Ui {
class cvImage;
}

class ThreadSafeLabel;

class cvImage : public QWidget
{
    Q_OBJECT

public:
    explicit cvImage(cv::Mat image = cv::Mat(), QWidget *parent = nullptr);
    ~cvImage();

    void update(cv::Mat &image);

public slots:
    void resizeEvent(QResizeEvent* event);
    //void paintEvent(QPaintEvent* event);

private:
    QGridLayout *gridLayout;
    ThreadSafeLabel *imgLabel;

    QTimer * timer;
    QPixmap pixmap;
    cv::Mat image;
    std::mutex updateMtx;
};

#endif // CVIMAGE_H
