#include "imusubtab.h"

MyEventFilter::MyEventFilter(QTimer * updateTimer, QObject *parent)
    : QObject(parent)
    , updateTimer_(updateTimer)
{
}

bool MyEventFilter::eventFilter(QObject *watched, QEvent *event)
{
    if (event->type() == QEvent::WindowDeactivate) {
        //auto widget = dynamic_cast<QWidget *>(watched);
        qDebug() << "WindowDeactivate\n";
        updateTimer_->stop();
        return true;
    } else if (event->type() == QEvent::WindowActivate) {
        //auto widget = dynamic_cast<QWidget *>(watched);
        qDebug() << "WindowActivate\n";
        updateTimer_->start(20);
        return true;
    } else if (event->type() == QEvent::FocusIn) {
        //auto widget = dynamic_cast<QWidget *>(watched);
        qDebug() << "FocusIn\n";
        updateTimer_->start(20);
        return true;
    } else if (event->type() == QEvent::FocusOut) {
        //auto widget = dynamic_cast<QWidget *>(watched);
        qDebug() << "FocusOut\n";
        updateTimer_->stop();
        return true;
    /*} else if (event->type() == QEvent::Enter) {
        //auto widget = dynamic_cast<QWidget *>(watched);
        qDebug() << "Enter\n";
        updateTimer_->start(20);
        //widget->show();
        return true;
    } else if (event->type() == QEvent::Leave) {
        //auto widget = dynamic_cast<QWidget *>(watched);
        qDebug() << "Leave\n";
        updateTimer_->stop();
        return true;*/
    } else {
        return QObject::eventFilter(watched, event);
    }
}

ImuSubTab::ImuSubTab(uint16_t sampleRate, QWidget *parent) :
    QWidget(parent),
    sample_rate(sampleRate),
    ui(new Ui::ImuTab)
{
    ui->setupUi(this);

    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    int graphIndex = 0;
    ui->plot->addGraph();
    ui->plot->graph(graphIndex)->setPen(QPen(Qt::red));
    ui->plot->graph(graphIndex)->setName("X");
    plotBuffers.emplace_back(sample_rate * buffer_length_seconds);
    graphIndex++;

    ui->plot->addGraph();
    ui->plot->graph(graphIndex)->setPen(QPen(Qt::green));
    ui->plot->graph(graphIndex)->setName("Y");
    plotBuffers.emplace_back(sample_rate * buffer_length_seconds);
    graphIndex++;

    ui->plot->addGraph();
    ui->plot->graph(graphIndex)->setPen(QPen(Qt::blue));
    ui->plot->graph(graphIndex)->setName("Z");
    plotBuffers.emplace_back(sample_rate * buffer_length_seconds);
    graphIndex++;

    QFont legendFont = font();
    legendFont.setPointSize(9);

    ui->plot->legend->setVisible(true);
    ui->plot->legend->setFont(legendFont);
    ui->plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft|Qt::AlignTop);
    ui->plot->legend->setBrush(QBrush(QColor(255,255,255,230)));
    ui->plot->xAxis->setLabel("Time [s]");
    //ui->plot->yAxis->setLabel("Angular Velocity [deg/s]");

    ui->plot->yAxis->setRange(-20, 130);

    ui->plot->xAxis->grid()->setSubGridVisible(true);
    ui->plot->yAxis->grid()->setSubGridVisible(true);

    ui->plot->axisRect()->setRangeZoom(Qt::Orientations(Qt::Horizontal | Qt::Vertical));

    // setup policy and connect slot for context menu popup:
    ui->plot->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->plot, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));

    plotUpdateTimer = new QTimer(this);
    plotUpdateTimer->start(1000 / (sample_rate / samples_to_collate_before_plot_update));
    //connect(plotUpdateTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(plotUpdateTimer, &QTimer::timeout, this, [=]() {
          plotUpdateTimerEvent();
     });

    // Install event filter to disable updating when plot is not in focus
    /*auto myEventFilter = new MyEventFilter(plotUpdateTimer, this);
    ui->plot->installEventFilter(myEventFilter);*/

    connect(ui->plotOption, qOverload<int>(&QComboBox::currentIndexChanged), this, [=](int index) {
          PlotOptionChanged(index);
    });
}

ImuSubTab::~ImuSubTab()
{    
    delete plotUpdateTimer;
}

void ImuSubTab::plotUpdateTimerEvent()
{
    if (!ui->updateEnabled->isChecked()) return;

    std::lock_guard<std::mutex> lock(plotBuffersMutex);
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();

    size_t plotsToShow = 0;
    if (ui->plotOption->currentIndex() < plotOptions.size() && plotOptions.at(ui->plotOption->currentIndex()).second)
        plotsToShow = 6;
    else
        plotsToShow = 3;

    plotsToShow = std::min(plotsToShow, plotBuffers.size());

    double visible_range = visible_buffer_length_seconds;
    if (!ui->autoScalingX->isChecked()) {
        visible_range = ui->plot->xAxis->range().upper - ui->plot->xAxis->range().lower;
    }

    for (size_t i = 0; i < plotsToShow; i++) {
        if (plotBuffers[i].times().size() > 0) {
            ui->plot->graph(i)->setData(plotBuffers[i].times(), plotBuffers[i].values(), true);

            auto first_timestamp = plotBuffers[i].times().first();
            if (first_timestamp < x_min) {
                x_min = first_timestamp;
            }
            auto last_timestamp = plotBuffers[i].times().back();
            if (last_timestamp > x_max) {
                x_max = last_timestamp;
            }

            int visible_window_values = visible_range * sample_rate;
            visible_window_values = std::min(visible_window_values, plotBuffers[i].values().size());
            auto max_y = std::max_element(std::end(plotBuffers[i].values()) - visible_window_values, std::end(plotBuffers[i].values()));
            auto min_y = std::min_element(std::end(plotBuffers[i].values()) - visible_window_values, std::end(plotBuffers[i].values()));
            if (*min_y < y_min) y_min = *min_y;
            if (*max_y > y_max) y_max = *max_y;
        }
    }
    x_min = std::max(x_min, 0.0);

    if (ui->autoScalingX->isChecked() || ui->autoScalingY->isChecked() || ui->follow->isChecked()) {
        // Compute the axis scaling
        if (ui->follow->isChecked()) {
            double x_min2 = x_max - visible_range;
            x_min = std::max(x_min, x_min2);
            x_max = x_min + visible_range;
        }

        if (ui->follow->isChecked())
            ui->plot->xAxis->setRange(x_min, x_max);
        if (ui->autoScalingY->isChecked() && y_max > y_min)
            ui->plot->yAxis->setRange(y_min, y_max);
    }
    //accelerometerTab->plot->rescaleAxes(); // instead of performing automatic scaling

    ui->plot->replot();
}

void ImuSubTab::AddMeasurement(int type_index, double timestamp, Eigen::Vector3f measurement)
{
    std::lock_guard<std::mutex> lock(plotBuffersMutex);
    if (ui->plotOption->currentIndex() == type_index) {
        plotBuffers[0].add(timestamp, measurement[0]);
        plotBuffers[1].add(timestamp, measurement[1]);
        plotBuffers[2].add(timestamp, measurement[2]);
    }
}

void ImuSubTab::AddSecondMeasurement(int type_index, double timestamp, Eigen::Vector3f measurement)
{
    std::lock_guard<std::mutex> lock(plotBuffersMutex);
    if (ui->plotOption->currentIndex() == type_index && plotBuffers.size() >= 6) {
        plotBuffers[3].add(timestamp, measurement[0]);
        plotBuffers[4].add(timestamp, measurement[1]);
        plotBuffers[5].add(timestamp, measurement[2]);
    }
}

void ImuSubTab::AddMeasurement(int type_index, double timestamp, Eigen::Vector3d measurement)
{
    std::lock_guard<std::mutex> lock(plotBuffersMutex);
    if (ui->plotOption->currentIndex() == type_index) {
        plotBuffers[0].add(timestamp, measurement[0]);
        plotBuffers[1].add(timestamp, measurement[1]);
        plotBuffers[2].add(timestamp, measurement[2]);
    }
}

void ImuSubTab::AddSecondMeasurement(int type_index, double timestamp, Eigen::Vector3d measurement)
{
    std::lock_guard<std::mutex> lock(plotBuffersMutex);
    if (ui->plotOption->currentIndex() == type_index && plotBuffers.size() >= 6) {
        plotBuffers[3].add(timestamp, measurement[0]);
        plotBuffers[4].add(timestamp, measurement[1]);
        plotBuffers[5].add(timestamp, measurement[2]);
    }
}

void ImuSubTab::AddPlotOption(QString name, QString yAxisLabel, bool withSecondMeasurement)
{
    plotOptions.emplace_back(std::pair<QString, QString>(name, yAxisLabel), withSecondMeasurement);
    ui->plotOption->addItem(name);
    ui->plot->yAxis->setLabel(yAxisLabel);

    if (withSecondMeasurement)
        AddExtraMeasurementPlot(ui->plotOption->count() == 1); // only make visible if it is the first plot added
}

void ImuSubTab::AddExtraMeasurementPlot(bool visible)
{
    int graphIndex = 3;
    ui->plot->addGraph();
    ui->plot->graph(graphIndex)->setPen(QPen(QBrush(Qt::red), 1, Qt::DashLine));
    ui->plot->graph(graphIndex)->setName("X");
    ui->plot->graph(graphIndex)->setVisible(visible);
    if (!visible)
        ui->plot->graph(graphIndex)->removeFromLegend();
    plotBuffers.emplace_back(sample_rate * buffer_length_seconds);
    graphIndex++;

    ui->plot->addGraph();
    ui->plot->graph(graphIndex)->setPen(QPen(QBrush(Qt::green), 1, Qt::DashLine));
    ui->plot->graph(graphIndex)->setName("Y");
    ui->plot->graph(graphIndex)->setVisible(visible);
    if (!visible)
        ui->plot->graph(graphIndex)->removeFromLegend();
    plotBuffers.emplace_back(sample_rate * buffer_length_seconds);
    graphIndex++;

    ui->plot->addGraph();
    ui->plot->graph(graphIndex)->setPen(QPen(QBrush(Qt::blue), 1, Qt::DashLine));
    ui->plot->graph(graphIndex)->setName("Z");
    ui->plot->graph(graphIndex)->setVisible(visible);
    if (!visible)
        ui->plot->graph(graphIndex)->removeFromLegend();
    plotBuffers.emplace_back(sample_rate * buffer_length_seconds);
    graphIndex++;
}

void ImuSubTab::ChangeLegend(int index, QString legend)
{
    auto * graph = ui->plot->graph(index);
    if (graph) {
        graph->setName(legend);
    }
}

void ImuSubTab::PlotOptionChanged(int index)
{
    if (static_cast<size_t>(index) >= plotOptions.size()) return;

    ui->plot->yAxis->setLabel(plotOptions.at(index).first.second);

    if (plotOptions.at(index).second) {
        for (size_t i = 3; i < plotBuffers.size(); i++) {
            ui->plot->graph(i)->setVisible(true);
            ui->plot->graph(i)->addToLegend();
        }
    } else {
        for (size_t i = 3; i < plotBuffers.size(); i++) {
            ui->plot->graph(i)->setVisible(false);
            ui->plot->graph(i)->removeFromLegend();
        }
    }

    for (size_t i = 0; i < plotBuffers.size(); i++) {
        plotBuffers[i].clear();
    }
}

void ImuSubTab::contextMenuRequest(QPoint pos)
{
  QMenu *menu = new QMenu(this);
  menu->setAttribute(Qt::WA_DeleteOnClose);

  menu->addAction("Save Plot", this, [=]() {
      QString fileName = QFileDialog::getSaveFileName(this,
              tr("Save plot"), "",
              tr("PDF (*.pdf);;PNG (*.png);;JPEG (*.jpg);;All Files (*)"));
      if (!fileName.isEmpty()) {
          QString extension = QFileInfo(fileName).completeSuffix();
          if (!extension.compare("png")) {
              ui->plot->savePng(fileName);
          }
          else if (!extension.compare("jpg")) {
              ui->plot->saveJpg(fileName);
          }
          else if (!extension.compare("pdf")) {
              ui->plot->savePdf(fileName);
          }
          else {
              ui->plot->savePdf(fileName + ".pdf");
          }
      }
   });

  menu->popup(ui->plot->mapToGlobal(pos));
}

PlotBuffer::PlotBuffer(size_t size)
    : size_(size)
{
    times_.reserve(size);
    values_.reserve(size);
}

void PlotBuffer::add(double time, double value)
{
    times_.append(time);
    if(times_.size() > static_cast<int>(size_)) {
        times_.remove(0, times_.size() - size_);
    }

    values_.append(value);
    if(values_.size() > static_cast<int>(size_)) {
        values_.remove(0, values_.size() - size_);
    }
}

void PlotBuffer::clear()
{
    times_.clear();
    values_.clear();
}
