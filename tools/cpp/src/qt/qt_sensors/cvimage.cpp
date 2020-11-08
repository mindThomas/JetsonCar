#include "cvimage.h"
//#include "ui_cvimage.h"

class ThreadSafeLabel : public QLabel
{
    std::mutex * mtx;

public:
    ThreadSafeLabel(std::mutex * mtx_, QWidget *parent) : QLabel(parent), mtx(mtx_)
    {
    }

public slots:
    void paintEvent(QPaintEvent* event)
    {
        mtx->lock();
        //if () {
        QLabel::paintEvent(event);
        mtx->unlock();
        //}
    }
};

cvImage::cvImage(cv::Mat image, QWidget *parent) :
    QWidget(parent)
{
    //ui->setupUi(this);
    //if (cvImage->objectName().isEmpty())
    //    cvImage->setObjectName(QString::fromUtf8("cvImage"));
    //((QWidget*)this)->resize(400, 300);
    gridLayout = new QGridLayout(this);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));

    imgLabel = new ThreadSafeLabel(&updateMtx, this);
    imgLabel->setObjectName(QString::fromUtf8("imgLabel"));
    //imgLabel->setMinimumSize(QSize(1, 1));

    //QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    //sizePolicy.setHorizontalStretch(0);
    //sizePolicy.setVerticalStretch(0);
    //sizePolicy.setHeightForWidth(imgLabel->sizePolicy().hasHeightForWidth());
    //sizePolicy.setRetainSizeWhenHidden(false);
    imgLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    gridLayout->addWidget(imgLabel, 0, 0, 1, 1);

    //connect(ui->imgLabel, &QLabel::paintEvent, std::bind(LabelPaintEvent, this, std::placeholders::_1));

    if (image.rows > 0) {
        update(image);
    }
}

cvImage::~cvImage()
{
    delete gridLayout;
    delete imgLabel;
}

void cvImage::update(cv::Mat &image)
{
    if (image.rows == 0) {        
        if (!imgLabel->isHidden())
            imgLabel->hide();
        return;
    }
    if (imgLabel->isHidden())
        imgLabel->show();

    //conversion from Mat to QImage
    cv::Mat dest;        
    if (image.channels() == 1) { // greyscale
        cv::cvtColor(image, dest, CV_GRAY2RGB);
    }
    else if (image.channels() == 3) { // BGR
        cv::cvtColor(image, dest, CV_BGR2RGB);
    }
    else if (image.channels() == 4) { // BGRA
        cv::cvtColor(image, dest, CV_BGRA2RGB);
    }
    else {
        return; // error
    }

    QImage image1 = QImage((uchar*) dest.data, dest.cols, dest.rows, dest.step, QImage::Format_RGB888);

    pixmap = QPixmap::fromImage(image1);

    if (updateMtx.try_lock()) {
        //show Qimage using QLabel with auto scaling
        int w = imgLabel->width();
        int h = imgLabel->height();

        imgLabel->setPixmap(pixmap.scaled(w, h, Qt::KeepAspectRatio));
        imgLabel->setMinimumSize(1, 1); // required to allow downscaling - this can also be enabled from within the QT Creator design
        updateMtx.unlock();
    }
}

void cvImage::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);

    if (updateMtx.try_lock()) {
        //show Qimage using QLabel with auto scaling
        int w = imgLabel->width();
        int h = imgLabel->height();
        imgLabel->setPixmap(pixmap.scaled(w, h, Qt::KeepAspectRatio));
        updateMtx.unlock();
    }
}

/*void cvImage::LabelPaintEvent(QPaintEvent* event)
{
    std::lock_guard<std::mutex> lock(updateMtx);
    ui->imgLabel->paintEvent(event);
}*/
