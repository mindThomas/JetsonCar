#include "testview.h"
#include "ui_testview.h"

#include <QCheckBox>
#include <QLabel>
#include <QMainWindow>
#include <QSlider>
#include <QWidget>
#include <QString>

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>

TestView::TestView(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TestView)
{
    ui->setupUi(this);

    count = 0;
    //connect(ui->pushButton, SIGNAL (clicked(bool)), this, SLOT (ButtonPress(bool)));
    connect(ui->pushButton, &QPushButton::clicked, this, &TestView::ButtonPress);
}

TestView::~TestView()
{
    delete ui;
}

void TestView::ButtonPress(bool pressed)
{
    std::stringstream ss;

    count++;    

    // Two ways of doing the same thing
    // Option 1:
    //ss << "Count = " << count;
    //ui->label->setText(QString::fromStdString(ss.str()));
    // Option 2:
    ui->label->setText(QString("Count = %1").arg(count));
}
