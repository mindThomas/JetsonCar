#include "imutab.h"
#include "ui_imutab.h"

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

ImuTab::ImuTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImuTab)
{
    ui->setupUi(this);

    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    // Current and duty
    int graphIndex = 0;
    ui->plot->addGraph();
    ui->plot->graph(graphIndex)->setPen(QPen(Qt::red));
    ui->plot->graph(graphIndex)->setName("Gyro X");
    graphIndex++;

    ui->plot->addGraph();
    ui->plot->graph(graphIndex)->setPen(QPen(Qt::green));
    ui->plot->graph(graphIndex)->setName("Gyro Y");
    graphIndex++;

    ui->plot->addGraph(ui->plot->xAxis, ui->plot->yAxis2);
    ui->plot->graph(graphIndex)->setPen(QPen(Qt::blue));
    ui->plot->graph(graphIndex)->setName("Gyro Z");
    graphIndex++;

    QFont legendFont = font();
    legendFont.setPointSize(9);

    ui->plot->legend->setVisible(true);
    ui->plot->legend->setFont(legendFont);
    ui->plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignRight|Qt::AlignTop);
    ui->plot->legend->setBrush(QBrush(QColor(255,255,255,230)));
    ui->plot->xAxis->setLabel("Time [s]");
    ui->plot->yAxis->setLabel("Angular Velocity [deg/s]");
    ui->plot->yAxis2->setLabel("Angle [deg]");

    ui->plot->yAxis->setRange(-20, 130);
    ui->plot->yAxis2->setRange(-0.2, 1.3);
    ui->plot->yAxis2->setVisible(true);

    ui->plot->xAxis->grid()->setSubGridVisible(true);
    ui->plot->yAxis->grid()->setSubGridVisible(true);

    ui->plot->axisRect()->setRangeZoom(Qt::Orientations(Qt::Horizontal /*| Qt::Vertical*/));

    mTimer = new QTimer(this);
    mTimer->start(20);
    //connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mTimer, &QTimer::timeout, this, [=]() {
          timerEvent();
     });

    auto myEventFilter = new MyEventFilter(mTimer, this);
    ui->plot->installEventFilter(myEventFilter);

    /* initialize random seed: */
    srand (time(NULL));
 }

ImuTab::~ImuTab()
{
    delete ui;
}

void ImuTab::timerEvent()
{
    double randomValue = double(rand() % 10000) / 100;
    appendDoubleAndTrunc(&X, X.back()+1, 200);
    appendDoubleAndTrunc(&Y, randomValue, 200);

    ui->plot->graph(0)->setData(X, Y);
    //ui->plot->rescaleAxes();
    ui->plot->replot();
}

void ImuTab::appendDoubleAndTrunc(QVector<double> *vec, double num, int maxSize)
{
    vec->append(num);

    if(vec->size() > maxSize) {
        vec->remove(0, vec->size() - maxSize);
    }
}
