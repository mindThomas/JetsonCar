#ifndef TESTTAB_H
#define TESTTAB_H

#include <QWidget>

#include <qmessagebox.h>

namespace Ui {
class TestTab;
}

class TestTab : public QWidget
{
    Q_OBJECT

public:
    explicit TestTab(QWidget *parent = nullptr);
    ~TestTab();    

private slots:
    void on_testButton_clicked();

    void on_roll_valueChanged(int value);
    void on_pitch_valueChanged(int value);
    void on_yaw_valueChanged(int value);

private:
    Ui::TestTab *ui;
};

#endif // TESTTAB_H
