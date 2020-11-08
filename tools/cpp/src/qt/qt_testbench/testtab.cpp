#include "testtab.h"
#include "ui_testtab.h"

TestTab::TestTab(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TestTab)
{
    ui->setupUi(this);

    ui->vesc3Dview->setMinimumWidth(200);
    ui->vesc3Dview->setRollPitchYaw(20, 20, 0);
    ui->vesc3Dview->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

    // Another way of linking an action using a Lambda function
    connect(ui->anotherButton, &QPushButton::clicked, [=]() {
        QMessageBox::information( this, "Test GUI", "You pressed the other button.", QMessageBox::Ok );
    });
}

TestTab::~TestTab()
{
    delete ui;
}

void TestTab::on_testButton_clicked()
{
    ui->label->setText(QString::fromUtf8("Button was pressed"));
    QMessageBox::information( this, "Test GUI", "You pressed the button!", QMessageBox::Ok );
    ui->label->setText(QString::fromUtf8("Press the button"));
}

void TestTab::on_roll_valueChanged(int value)
{
   ui->vesc3Dview->setRollPitchYaw(ui->roll->value(), ui->pitch->value(), ui->yaw->value());
}

void TestTab::on_pitch_valueChanged(int value)
{
    ui->vesc3Dview->setRollPitchYaw(ui->roll->value(), ui->pitch->value(), ui->yaw->value());
}
void TestTab::on_yaw_valueChanged(int value)
{
    ui->vesc3Dview->setRollPitchYaw(ui->roll->value(), ui->pitch->value(), ui->yaw->value());
}
