#include "QTOpenCVTest.h"
#include "ui_QTOpenCVTest.h"

#include <QMainWindow>
#include <QWidget>
#include <QLabel>

#include <iostream>
#include <string>
#include <stdlib.h>     /* srand, rand */

QTOpenCVTest::QTOpenCVTest(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::QTOpenCVTest)
{
    ui->setupUi(this);

    resetPicture();
    connect(ui->pushButton, &QPushButton::clicked, this, &QTOpenCVTest::resetPicture);

    timer = new QTimer(this);
    //connect(timer, &QTimer::timeout, this, &QTOpenCVTest::updatePicture);
    connect(timer, SIGNAL(timeout()), this, SLOT(updatePicture()));
    timer->start(20);
}

QTOpenCVTest::~QTOpenCVTest()
{
    delete timer;
    delete ui;
}

void QTOpenCVTest::resetPicture()
{
    image = cv::imread( std::string(__SOURCE_FOLDER__) + std::string("/test.png"), cv::IMREAD_COLOR ); // Read the file

    if (image.rows == 0) {
        std::cout << "Error loading image" << std::endl;
        return;
    }

    //conversion from Mat to QImage
    cv::Mat dest;
    cv::cvtColor(image, dest, cv::COLOR_BGR2RGB);
    QImage image1 = QImage((uchar*) dest.data, dest.cols, dest.rows, dest.step, QImage::Format_RGB888);

    pixmap = QPixmap::fromImage(image1);

    //show Qimage using QLabel with auto scaling
    int w = ui->imgLabel->width();
    int h = ui->imgLabel->height();
    ui->imgLabel->setPixmap(pixmap.scaled(w, h, Qt::KeepAspectRatio));

    ui->imgLabel->setMinimumSize(1, 1); // required to allow downscaling - this can also be enabled from within the QT Creator design
}

void QTOpenCVTest::updatePicture()
{
    if (image.rows == 0) return;

    int x = (image.cols-1) * ((float)rand() / RAND_MAX);
    int y = (image.rows-1) * ((float)rand() / RAND_MAX);

    uint8_t R = 255 * ((float)rand() / RAND_MAX);
    uint8_t G = 255 * ((float)rand() / RAND_MAX);
    uint8_t B = 255 * ((float)rand() / RAND_MAX);

    image.at<cv::Vec3b>(y,x)[0] = B; // Blue
    image.at<cv::Vec3b>(y,x)[1] = G; // Green
    image.at<cv::Vec3b>(y,x)[2] = R; // Red

    //conversion from Mat to QImage
    cv::Mat dest;
    cv::cvtColor(image, dest, cv::COLOR_BGR2RGB);
    QImage image1 = QImage((uchar*) dest.data, dest.cols, dest.rows, dest.step, QImage::Format_RGB888);

    pixmap = QPixmap::fromImage(image1);

    //show Qimage using QLabel with auto scaling
    int w = ui->imgLabel->width();
    int h = ui->imgLabel->height();
    ui->imgLabel->setPixmap(pixmap.scaled(w, h, Qt::KeepAspectRatio));
}

void QTOpenCVTest::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);

    //show Qimage using QLabel with auto scaling
    int w = ui->imgLabel->width();
    int h = ui->imgLabel->height();
    ui->imgLabel->setPixmap(pixmap.scaled(w, h, Qt::KeepAspectRatio));
}