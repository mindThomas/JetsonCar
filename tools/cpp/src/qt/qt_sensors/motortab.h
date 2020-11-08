#ifndef MOTORTAB_H
#define MOTORTAB_H

#include <QWidget>

namespace Ui {
class MotorTab;
}

class MotorTab : public QWidget
{
    Q_OBJECT

public:
    explicit MotorTab(QWidget *parent = nullptr);
    ~MotorTab();

private:
    Ui::MotorTab *ui;
};

#endif // MOTORTAB_H
