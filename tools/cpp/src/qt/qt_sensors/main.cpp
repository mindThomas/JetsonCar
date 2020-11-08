#include "imu3dview.h"
#include "qcustomplot.h"

#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    // Set OpenGL Version information
    // Note: This format must be set before show() is called.
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setVersion(3, 3);

    // DPI settings
    // TODO: http://www.qcustomplot.com/index.php/support/forum/1344

    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QCoreApplication::setAttribute(Qt::AA_Use96Dpi);

#if 0
    QSettings set;
    bool scaleAuto = true;
    double scale = 1.0;

    if (set.contains("app_scale_auto")) {
        scaleAuto = set.value("app_scale_auto").toBool();
    } else {
        set.setValue("app_scale_auto", scaleAuto);
    }

    if (scaleAuto) {
        QApplication tmp(argc, argv);
        QRect rec = tmp.desktop()->screenGeometry();
        int height = rec.height();
        int width = rec.width();
        double ptFont = tmp.font().pointSizeF();
        if (ptFont < 0.0) {
            ptFont = tmp.font().pixelSize();
        }

        if (width > 3000 && height > 1700) {
            scale = 1.5;
        } else {
            if (ptFont > 11.0) {
                scale = ptFont / 11.0;
            }
        }

        set.setValue("app_scale_factor", scale);
    } else if (set.contains("app_scale_factor")) {
        scale = set.value("app_scale_factor").toDouble();
    }

    set.setValue("app_scale_factor", scale);

    if (scale > 1.01) {
        qputenv("QT_SCALE_FACTOR", QString::number(scale).toLocal8Bit());
    }
#endif
    // qcustomplot does not look well with different scaling. So force scale to be 1.
    qputenv("QT_SCALE_FACTOR", QString::number(1.0).toLocal8Bit());

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
