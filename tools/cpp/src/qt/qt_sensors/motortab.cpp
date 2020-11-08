#include "motortab.h"
#include "ui_motortab.h"

MotorTab::MotorTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MotorTab)
{
    ui->setupUi(this);
}

MotorTab::~MotorTab()
{
    delete ui;
}
