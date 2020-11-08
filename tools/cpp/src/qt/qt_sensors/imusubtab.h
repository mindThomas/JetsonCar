#ifndef IMUSUBTAB_H
#define IMUSUBTAB_H

#include <QWidget>
#include <QTimer>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <mutex>

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>
#include <QFileDialog>
#include "qcustomplot.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ui_imutab.h"

namespace Ui {
class ImuTab;
}

class PlotBuffer
{
public:
    PlotBuffer(size_t size);

    void add(double time, double value);
    void clear();

    const QVector<double> times() const { return times_; };
    const QVector<double> values() const { return values_; };

private:
    size_t size_;
    QVector<double> times_;
    QVector<double> values_;

};

class ImuSubTab : public QWidget
{
    Q_OBJECT

public:
    explicit ImuSubTab(uint16_t sampleRate, QWidget *parent = nullptr);
    ~ImuSubTab();

    void AddMeasurement(int type_index, double timestamp, Eigen::Vector3f measurement);
    void AddMeasurement(int type_index, double timestamp, Eigen::Vector3d measurement);
    void AddSecondMeasurement(int type_index, double timestamp, Eigen::Vector3f measurement);
    void AddSecondMeasurement(int type_index, double timestamp, Eigen::Vector3d measurement);
    void AddPlotOption(QString name, QString yAxisLabel, bool withSecondMeasurement = false);
    void ChangeLegend(int index, QString legend);

private:
    const uint8_t samples_to_collate_before_plot_update{5};
    const uint16_t sample_rate{100};
    const double buffer_length_seconds{1000};
    const double visible_buffer_length_seconds{5};

public:
    Ui::ImuTab * ui;

private:        
    QTimer *plotUpdateTimer;

    std::vector<PlotBuffer> plotBuffers;
    std::mutex plotBuffersMutex;

    std::vector<std::pair<std::pair<QString, QString>, bool>> plotOptions;

    void AddExtraMeasurementPlot(bool visible = true);

    void plotUpdateTimerEvent();
    void appendDoubleAndTrunc(QVector<double> *vec, double num, int maxSize);

    void PlotOptionChanged(int index);

private slots:
    void contextMenuRequest(QPoint pos);
};

class MyEventFilter : public QObject
{
    Q_OBJECT
public:
    MyEventFilter(QTimer * updateTimer, QObject *parent = nullptr);

protected:
    virtual bool eventFilter(QObject *watched, QEvent *event) override;

private:
    QTimer * updateTimer_;
};

#endif // IMUTAB_H
