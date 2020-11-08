#ifndef IMUTAB_H
#define IMUTAB_H

#include <QWidget>
#include <QTimer>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

namespace Ui {
class ImuTab;
}

class ImuTab : public QWidget
{
    Q_OBJECT

public:
    explicit ImuTab(QWidget *parent = nullptr);
    ~ImuTab();

    Ui::ImuTab *ui;

private:    

    QTimer *mTimer;
    QVector<double> X;
    QVector<double> Y;

    void timerEvent();
    void appendDoubleAndTrunc(QVector<double> *vec, double num, int maxSize);
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
