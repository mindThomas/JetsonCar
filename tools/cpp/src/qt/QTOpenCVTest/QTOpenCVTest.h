#ifndef QTOPENCVTEST_H
#define QTOPENCVTEST_H

#include <QMainWindow>
#include <QTimer>
#include <QPixmap>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace Ui {
class QTOpenCVTest;
}

class QTOpenCVTest : public QMainWindow
{
    Q_OBJECT

public:
    explicit QTOpenCVTest(QWidget *parent = nullptr);
    ~QTOpenCVTest();

public slots:
    void resizeEvent(QResizeEvent* event);
    void resetPicture();
    void updatePicture();

private:
    Ui::QTOpenCVTest *ui;

    QTimer * timer;
    QPixmap pixmap;
    cv::Mat image;
};

#endif // QTOPENCVTEST_H
